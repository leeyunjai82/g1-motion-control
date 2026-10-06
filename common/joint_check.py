"""
joint_check.py — 모터 번호 확인 화면 (simulator.py 에 붙는 라우터, http://<pc>:8000/check)

목적: robots/<ROBOT>/robot.yaml 의 관절 맵(URDF 이름 → 모터 슬롯)이 실제 로봇과 같은지 슬롯별로 확인하고 기록.

두 가지 확인 방법
  1) 읽기 전용 (35 슬롯 전부, 다리 포함): rt/lowstate 각도를 실시간 표시하고 [기준 잡기] 이후 변화량(Δ)을 강조.
     로봇 관절을 손으로 움직이면 (FSM Damp 또는 [제어권 반납] 후) 어느 슬롯이 변하는지 보인다.
  2) 명령 이동 (팔·허리 슬롯만): 슬롯 하나를 ±5° 씩 움직인다. 기준 대비 최대 ±15° (서버에서 제한).
     실제로 움직인 관절 ↔ 3D 뷰어(:50003, robot.yaml 맵으로 그림)에서 움직인 관절이 같은지 본다.
     헤드 슬롯은 arm_server 명령이 없어 1) 로만 확인.

결과는 [저장] → robots/<ROBOT>/joint_check_<날짜시각>.json (슬롯·이름·판정·메모·mode_machine)

명령은 모두 arm_server(:50022) 경유 — rt/arm_sdk 단독 점유 원칙 그대로.
"""
import json
import os
import threading
import time
import urllib.request

import numpy as np
from fastapi import APIRouter, HTTPException
from fastapi.responses import HTMLResponse

import robot_env

ARM_SERVER = "http://localhost:50022"
STEP_DEG = 5.0          # 버튼 한 번 이동량
MAX_OFFSET_DEG = 15.0   # 기준 대비 최대 이동 (서버에서 제한)
MOVE_SEC = 1.0

J = robot_env.JOINTS
N = int(J["motor_slots"])
ARM = [int(i) for i in J["arm"]]
WAIST = [int(i) for i in J["waist"]]
HEAD = [int(i) for i in (J.get("head") or [])]
SLOT_NAME = {int(v): str(k) for k, v in J["map"].items()}
SLOT_NAME[int(J["weight_slot"])] = "(arm_sdk weight 슬롯)"

router = APIRouter()

_ls = {"q": None, "mm": None, "t": 0.0, "temp": None}
_ls_lock = threading.Lock()
_sub_started = False
_base = {"q": None, "arm_t": None, "waist_t": None}
_offset = {}             # slot -> deg (명령 이동 누적)


def _arm(path, body=None, timeout=10.0):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(ARM_SERVER + path, data=data, method="POST" if body is not None else "GET",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def _ensure_sub():
    """rt/lowstate 구독 (읽기 전용). simulator lifespan 의 dds_init 이후 첫 호출에서 시작."""
    global _sub_started
    if _sub_started:
        return
    _sub_started = True
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    def loop():
        sub = ChannelSubscriber("rt/lowstate", LowState_)
        sub.Init()
        while True:
            m = sub.Read(1.0)
            if m is None:
                continue
            q = np.array([m.motor_state[i].q for i in range(N)], dtype=float)
            temp = [int(m.motor_state[i].temperature[0]) for i in range(N)]
            with _ls_lock:
                _ls.update(q=q, mm=int(m.mode_machine), t=time.time(), temp=temp)
    threading.Thread(target=loop, daemon=True).start()


def _capture_base():
    with _ls_lock:
        q = None if _ls["q"] is None else _ls["q"].copy()
    p = _arm("/pose")
    _base.update(q=q, arm_t=list(p["arm_target_deg"]), waist_t=list(p["waist_target_deg"]))
    _offset.clear()


@router.get("/check", response_class=HTMLResponse)
def check_page():
    mode = "가상 (ROBOT_SIM, DDS 도메인 1)" if robot_env.SIM else "실기 (DDS 도메인 0)"
    return (HTML.replace("__ROBOT__", robot_env.ROBOT.upper()).replace("__MODE__", mode)
            .replace("__REAL__", "0" if robot_env.SIM else "1")
            .replace("__STEP__", str(STEP_DEG)).replace("__MAX__", str(MAX_OFFSET_DEG)))


@router.get("/check/state")
def check_state():
    _ensure_sub()
    with _ls_lock:
        q = None if _ls["q"] is None else _ls["q"].copy()
        mm, age, temp = _ls["mm"], time.time() - _ls["t"], _ls["temp"]
    rows = []
    for s in range(N):
        kind = ("arm" if s in ARM else "waist" if s in WAIST else "head" if s in HEAD
                else "weight" if s == int(J["weight_slot"]) else "leg" if s in SLOT_NAME else "none")
        qd = None if q is None else round(float(np.degrees(q[s])), 2)
        dd = None
        if q is not None and _base["q"] is not None:
            dd = round(float(np.degrees(q[s] - _base["q"][s])), 2)
        rows.append({"slot": s, "name": SLOT_NAME.get(s, ""), "kind": kind, "q_deg": qd, "delta_deg": dd,
                     "offset_deg": _offset.get(s, 0.0), "temp": None if temp is None else temp[s]})
    try:
        arm = _arm("/status", timeout=0.5)
    except Exception as e:
        arm = {"error": str(e)}
    return {"robot": robot_env.ROBOT, "sim": robot_env.SIM, "mode_machine": mm, "lowstate_age_s": round(age, 2),
            "has_base": _base["arm_t"] is not None, "arm": arm, "rows": rows}


@router.post("/check/baseline")
def check_baseline():
    _ensure_sub()
    _capture_base()
    return {"ok": True}


@router.post("/check/jog")
def check_jog(slot: int, delta: float):
    if slot not in ARM and slot not in WAIST:
        raise HTTPException(400, f"슬롯 {slot} 은 명령 이동 대상 아님 (팔/허리만) — 손으로 움직여 읽기 전용으로 확인")
    if abs(delta) > STEP_DEG + 1e-6:
        raise HTTPException(400, f"한 번에 ±{STEP_DEG}° 까지")
    if _base["arm_t"] is None:
        _capture_base()
    st = _arm("/status", timeout=1.0)
    if st.get("mode") != "hold" or float(st.get("weight", 0)) < 0.99:
        raise HTTPException(409, "arm_server 가 hold(weight 1) 아님 — [제어권 잡기] 먼저")
    new = float(np.clip(_offset.get(slot, 0.0) + delta, -MAX_OFFSET_DEG, MAX_OFFSET_DEG))
    _offset[slot] = new
    if slot in ARM:
        deg = [_base["arm_t"][i] + _offset.get(s, 0.0) for i, s in enumerate(ARM)]
        _arm("/joints", {"deg": deg, "duration": MOVE_SEC}, timeout=MOVE_SEC + 10)
    else:
        w = [_base["waist_t"][i] + _offset.get(s, 0.0) for i, s in enumerate(WAIST)]
        _arm("/waist", {"yaw": w[0], "roll": w[1], "pitch": w[2], "duration": MOVE_SEC}, timeout=MOVE_SEC + 10)
    return {"ok": True, "slot": slot, "offset_deg": new}


@router.post("/check/reset")
def check_reset():
    """명령 이동을 모두 기준 자세로 되돌린다."""
    if _base["arm_t"] is None:
        return {"ok": True}
    _offset.clear()
    _arm("/joints", {"deg": _base["arm_t"], "duration": 2.0}, timeout=15)
    w = _base["waist_t"]
    _arm("/waist", {"yaw": w[0], "roll": w[1], "pitch": w[2], "duration": 2.0}, timeout=15)
    return {"ok": True}


@router.post("/check/hold")
def check_hold():
    return _arm("/hold", {"duration": 2.0}, timeout=15)


@router.post("/check/release")
def check_release():
    _offset.clear()
    _base.update(arm_t=None, waist_t=None)
    return _arm("/release", {"duration": 2.0}, timeout=20)


@router.post("/check/save")
def check_save(body: dict):
    with _ls_lock:
        mm = _ls["mm"]
    out = {"robot": robot_env.ROBOT, "sim": robot_env.SIM, "time": time.strftime("%Y-%m-%d %H:%M:%S"),
           "mode_machine": mm, "results": body.get("results", [])}
    path = os.path.join(robot_env.ROBOT_DIR, f"joint_check_{time.strftime('%Y%m%d_%H%M%S')}"
                        f"{'_sim' if robot_env.SIM else ''}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    return {"ok": True, "path": path}


HTML = r"""<!DOCTYPE html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>__ROBOT__ Joint Check</title>
<style>
:root{--bg:#14171c;--card:#1d2229;--fg:#e6e9ee;--dim:#8a94a3;--ok:#4cc38a;--bad:#ef6b6b;--hi:#f5b84d}
body{margin:0;background:var(--bg);color:var(--fg);font:13px system-ui,sans-serif}
.top{padding:10px 16px;border-bottom:1px solid #2a313b;display:flex;gap:14px;align-items:center;flex-wrap:wrap}
.top b{font-size:16px}.badge{padding:3px 8px;border-radius:5px;font-weight:600}
.real{background:#7a2e2e}.sim{background:#24507a}
.wrap{display:grid;grid-template-columns:minmax(560px,1fr) 1fr;gap:12px;padding:12px}
@media(max-width:1100px){.wrap{grid-template-columns:1fr}}
.card{background:var(--card);border-radius:8px;padding:10px}
button{background:#2b3440;color:var(--fg);border:0;border-radius:5px;padding:6px 9px;margin:2px;cursor:pointer}
button:disabled{opacity:.35;cursor:default}
table{width:100%;border-collapse:collapse}th,td{padding:3px 5px;border-bottom:1px solid #2a313b;text-align:left}
th{color:var(--dim);font-weight:500}tr.moved td{background:#3a3220}td.num{text-align:right;font-variant-numeric:tabular-nums}
tr.k-leg td:first-child,tr.k-none td:first-child{color:var(--dim)}
select,input[type=text]{background:#11151a;color:var(--fg);border:1px solid #2a313b;border-radius:4px;padding:2px 4px}
iframe{width:100%;height:640px;border:0;border-radius:6px;background:#000}.note{color:var(--dim);line-height:1.5}
</style></head><body>
<div class="top"><b>__ROBOT__ 모터 번호 확인</b>
 <span class="badge" id="mb">__MODE__</span>
 <span id="hdr"></span>
 <a href="/" style="color:#5aa9ff">← 모션 에디터</a></div>
<div class="wrap"><div class="card">
 <div>
  <button onclick="api('/check/hold')">제어권 잡기 (hold)</button>
  <button onclick="api('/check/release')">제어권 반납 (손으로 움직이기)</button>
  <button onclick="api('/check/baseline')">기준 잡기 (Δ=0)</button>
  <button onclick="api('/check/reset')">명령 이동 원위치</button>
  <button onclick="save()">결과 저장</button>
 </div>
 <p class="note">
  · <b>읽기 전용</b>: [기준 잡기] 후 관절을 손으로 움직이면 Δ 가 변한 슬롯이 노랗게 표시됩니다 (다리·헤드 포함, 제어권 반납 또는 FSM Damp 상태에서).<br>
  · <b>명령 이동</b> (팔·허리): 한 번에 __STEP__°, 기준 대비 최대 ±__MAX__°. 실제 로봇에서 움직인 관절과 오른쪽 3D 에서 움직인 관절(robot.yaml 이름)이 같으면 ✓.<br>
  · 실기 모드는 로봇을 매달거나 지지한 상태에서, 주변을 비우고 사용하세요.</p>
 <div id="msg" class="note"></div>
 <table><thead><tr><th>슬롯</th><th>robot.yaml 이름</th><th class="num">각도°</th><th class="num">Δ°</th><th>명령</th><th>판정</th><th>메모</th></tr></thead>
 <tbody id="tb"></tbody></table>
</div><div class="card"><iframe id="dash"></iframe></div></div>
<script>
const REAL=__REAL__, host=location.hostname;
document.getElementById('mb').className='badge '+(REAL?'real':'sim');
document.getElementById('dash').src=`http://${host}:50003/dashboard`;
const res={};let built=false;
async function api(u){try{const r=await fetch(u,{method:'POST'});const d=await r.json();
  document.getElementById('msg').textContent=u+' → '+(r.ok?'OK':JSON.stringify(d.detail||d));}catch(e){document.getElementById('msg').textContent=u+' 실패 '+e;}}
async function jog(s,d){await api(`/check/jog?slot=${s}&delta=${d}`);}
function build(rows){const tb=document.getElementById('tb');tb.innerHTML='';
  rows.forEach(r=>{const tr=document.createElement('tr');tr.id='r'+r.slot;tr.className='k-'+r.kind;
    const can=(r.kind==='arm'||r.kind==='waist');
    tr.innerHTML=`<td>${r.slot}</td><td>${r.name||'—'} <span class="note">${r.kind}</span></td>
     <td class="num" id="q${r.slot}"></td><td class="num" id="d${r.slot}"></td>
     <td>${can?`<button onclick="jog(${r.slot},-__STEP__)">−</button><button onclick="jog(${r.slot},__STEP__)">+</button> <span class="note" id="o${r.slot}"></span>`:'<span class="note">손으로</span>'}</td>
     <td>${(r.name&&r.kind!=='weight')?`<select id="j${r.slot}"><option value="">-</option><option value="ok">✓ 맞음</option><option value="ng">✗ 다름</option></select>`:''}</td>
     <td>${(r.name&&r.kind!=='weight')?`<input type="text" id="m${r.slot}" size="16" placeholder="실제로 움직인 관절">`:''}</td>`;
    tb.appendChild(tr);});built=true;}
async function poll(){try{const d=await(await fetch('/check/state')).json();
  if(!built)build(d.rows);
  const a=d.arm||{};
  document.getElementById('hdr').textContent=`mode_machine=${d.mode_machine??'?'} · lowstate ${d.lowstate_age_s<0.5?'수신 중':'없음'} · arm_server ${a.mode||a.error||'?'} weight ${a.weight!==undefined?(+a.weight).toFixed(2):'?'}`+(d.has_base?'':' · 기준 없음');
  d.rows.forEach(r=>{const q=document.getElementById('q'+r.slot);if(!q)return;
    q.textContent=r.q_deg===null?'':r.q_deg.toFixed(1);
    const dd=document.getElementById('d'+r.slot);dd.textContent=r.delta_deg===null?'':r.delta_deg.toFixed(1);
    document.getElementById('r'+r.slot).classList.toggle('moved',r.delta_deg!==null&&Math.abs(r.delta_deg)>2);
    const o=document.getElementById('o'+r.slot);if(o)o.textContent=r.offset_deg?`(${r.offset_deg>0?'+':''}${r.offset_deg}°)`:'';});
  window._mm=d.mode_machine;}catch(e){}}
async function save(){const results=[];document.querySelectorAll('[id^=j]').forEach(el=>{const s=+el.id.slice(1);
  results.push({slot:s,name:(document.querySelector('#r'+s+' td:nth-child(2)').firstChild.textContent||'').trim(),
    judge:el.value,memo:document.getElementById('m'+s).value});});
  const r=await fetch('/check/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({results})});
  const d=await r.json();document.getElementById('msg').textContent='저장: '+(d.path||JSON.stringify(d));}
poll();setInterval(poll,300);
</script></body></html>"""
