#!/usr/bin/env python3
"""
G1 URDF 실시간 모터 시각화 서버
- URDF/STL을 assets에서 직접 서빙
- DDS LowState로 실시간 관절각 스트리밍 (SSE)
"""

import os
import sys
import json
import asyncio
import numpy as np
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse, FileResponse

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

ASSETS_DIR = os.path.join(current_dir, 'assets', 'g1')
URDF_PATH  = os.path.join(ASSETS_DIR, 'g1_29dof_rev_1_0.urdf')
MESH_DIR   = os.path.join(ASSETS_DIR, 'meshes')

# ==========================================
# 로봇 연결
# ==========================================
try:
    from ctrl.robot_arm import G1_29_ArmController
    ctrl = G1_29_ArmController(motion_mode=False, simulation_mode=False)
    ROBOT_AVAILABLE = True
    print("[시스템] 로봇 연결 성공")
except Exception as e:
    ROBOT_AVAILABLE = False
    ctrl = None
    print(f"[경고] 로봇 없음 (시뮬레이션 모드): {e}")

# ==========================================
# URDF 관절명 → 모터 인덱스 매핑
# ==========================================
JOINT_TO_MOTOR = {
    'left_hip_pitch_joint':      0,
    'left_hip_roll_joint':       1,
    'left_hip_yaw_joint':        2,
    'left_knee_joint':           3,
    'left_ankle_pitch_joint':    4,
    'left_ankle_roll_joint':     5,
    'right_hip_pitch_joint':     6,
    'right_hip_roll_joint':      7,
    'right_hip_yaw_joint':       8,
    'right_knee_joint':          9,
    'right_ankle_pitch_joint':  10,
    'right_ankle_roll_joint':   11,
    'waist_yaw_joint':          12,
    'waist_roll_joint':         13,
    'waist_pitch_joint':        14,
    'left_shoulder_pitch_joint':  15,
    'left_shoulder_roll_joint':   16,
    'left_shoulder_yaw_joint':    17,
    'left_elbow_joint':           18,
    'left_wrist_roll_joint':      19,
    'left_wrist_pitch_joint':     20,
    'left_wrist_yaw_joint':       21,
    'right_shoulder_pitch_joint': 22,
    'right_shoulder_roll_joint':  23,
    'right_shoulder_yaw_joint':   24,
    'right_elbow_joint':          25,
    'right_wrist_roll_joint':     26,
    'right_wrist_pitch_joint':    27,
    'right_wrist_yaw_joint':      28,
}

app = FastAPI(title="G1 URDF Viewer")

# ==========================================
# API 엔드포인트
# ==========================================
@app.get('/api/urdf')
def get_urdf():
    return FileResponse(URDF_PATH, media_type='text/xml')

@app.get('/api/meshes')
def list_meshes():
    files = [f for f in os.listdir(MESH_DIR) if f.lower().endswith('.stl')]
    return {'files': files}

@app.get('/api/mesh/{filename}')
def get_mesh(filename: str):
    path = os.path.join(MESH_DIR, filename)
    if not os.path.exists(path):
        for f in os.listdir(MESH_DIR):
            if f.lower() == filename.lower():
                path = os.path.join(MESH_DIR, f)
                break
    return FileResponse(path, media_type='application/octet-stream')

@app.get('/api/joint_states')
async def joint_states():
    async def gen():
        while True:
            if ROBOT_AVAILABLE and ctrl:
                q   = ctrl.get_current_motor_q()
                imu = ctrl.get_imu_rpy().tolist()
            else:
                q   = np.zeros(35)
                imu = [0.0, 0.0, 0.0]

            data = {j: float(q[i]) for j, i in JOINT_TO_MOTOR.items()}
            data['_imu']       = imu
            data['_connected'] = ROBOT_AVAILABLE
            yield f"data: {json.dumps(data)}\n\n"
            await asyncio.sleep(0.05)  # 20Hz

    return StreamingResponse(
        gen(),
        media_type='text/event-stream',
        headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'}
    )

@app.get('/api/status')
def status():
    return {'connected': ROBOT_AVAILABLE}

# ==========================================
# HTML
# ==========================================
HTML_PAGE = r"""<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>G1 URDF Viewer</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0a0a0f;color:#d0d0d8;font-family:'Segoe UI',system-ui,sans-serif;display:flex;flex-direction:column;height:100vh;overflow:hidden;font-size:13px}
header{background:#0f0f1a;border-bottom:1px solid #13bfd530;padding:0 16px;display:flex;align-items:center;gap:12px;height:44px;flex-shrink:0}
header h1{font-size:14px;color:#13bfd5;font-weight:600;letter-spacing:.5px}
.badge{font-size:10px;padding:2px 8px;border-radius:10px;background:#1a1a2a;color:#555;border:1px solid #222}
.badge.ok{color:#4caf80;border-color:#4caf5044}
.badge.live{color:#f9c300;border-color:#f9c30044;animation:pulse 1.5s infinite}
.badge.err{color:#ff6666;border-color:#ff444444}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.5}}
.main{display:flex;flex:1;overflow:hidden;min-height:0}
.left{width:210px;background:#0d0d16;border-right:1px solid #1a1a28;display:flex;flex-direction:column;flex-shrink:0;overflow-y:auto}
.sec{padding:10px 12px;border-bottom:1px solid #1a1a28}
.sec-title{font-size:10px;text-transform:uppercase;color:#444;letter-spacing:1.2px;margin-bottom:8px}
.tb{display:flex;align-items:center;gap:7px;width:100%;padding:6px 9px;background:#12121e;border:1px solid #1e1e2e;border-radius:7px;color:#666;font-size:11px;cursor:pointer;margin-bottom:5px;transition:all .15s;text-align:left}
.tb:hover{border-color:#13bfd533;color:#99d5db}
.tb.on{background:#13bfd510;border-color:#13bfd555;color:#13bfd5}
.tb.live-on{background:#f9c30010;border-color:#f9c30055;color:#f9c300}
.tb .ic{width:13px;flex-shrink:0;text-align:center;font-size:11px}
.info-row{display:flex;justify-content:space-between;font-size:11px;padding:2px 0;color:#444}
.info-val{color:#666}
.imu-row{font-size:11px;padding:3px 0;color:#2a6a6a}
.pose-row{display:flex;gap:5px;margin-top:5px}
.pose-btn{flex:1;padding:5px;background:#12121e;border:1px solid #1e1e2e;border-radius:6px;color:#555;font-size:10px;cursor:pointer;text-align:center;transition:all .15s}
.pose-btn:hover{border-color:#13bfd533;color:#99d5db}
.status-dot{width:7px;height:7px;border-radius:50%;background:#333;flex-shrink:0}
.status-dot.on{background:#4caf80;box-shadow:0 0 5px #4caf8088}
.status-dot.live{background:#f9c300;box-shadow:0 0 5px #f9c30088}
.status-dot.err{background:#ff4444}
.viewport{flex:1;position:relative;overflow:hidden;background:#0a0a0f}
canvas#cv{display:block;width:100%!important;height:100%!important}
.hud{position:absolute;bottom:10px;left:10px;font-size:10px;color:#282838;pointer-events:none;line-height:1.9}
.load-overlay{position:absolute;inset:0;background:#0a0a0fdd;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:14px}
.load-title{font-size:15px;color:#13bfd5;font-weight:500}
.pbar-bg{width:280px;height:5px;background:#1a1a28;border-radius:3px}
.pbar-fill{height:100%;background:#13bfd5;border-radius:3px;transition:width .07s;width:0%}
.pbar-text{font-size:11px;color:#555}
.tooltip{position:absolute;background:#14141e;border:1px solid #2a2a3a;border-radius:6px;padding:5px 10px;font-size:11px;color:#aaa;pointer-events:none;z-index:100;display:none}
.right{width:275px;background:#0d0d16;border-left:1px solid #1a1a28;display:flex;flex-direction:column;flex-shrink:0}
.right-head{padding:8px 12px;border-bottom:1px solid #1a1a28;display:flex;align-items:center;justify-content:space-between;gap:7px}
.right-head b{font-size:12px;font-weight:600;color:#aaa;white-space:nowrap}
.search-box{flex:1;background:#12121e;border:1px solid #1e1e2e;border-radius:5px;padding:4px 8px;color:#aaa;font-size:11px;outline:none;min-width:0}
.search-box::placeholder{color:#2a2a3a}
.search-box:focus{border-color:#13bfd544}
.jcount{font-size:10px;color:#444;white-space:nowrap}
.joints{flex:1;overflow-y:auto;padding:7px}
.no-joint{text-align:center;color:#2a2a3a;font-size:11px;padding:40px 16px;line-height:1.8}
.group-header{display:flex;align-items:center;gap:6px;padding:5px 4px;cursor:pointer;color:#444;font-size:10px;text-transform:uppercase;letter-spacing:.8px;user-select:none;margin-top:3px}
.group-header:hover{color:#666}
.garr{font-size:8px;transition:transform .15s;flex-shrink:0}
.garr.open{transform:rotate(90deg)}
.group-body{overflow:hidden}
.ji{margin-bottom:5px;background:#12121e;border-radius:7px;padding:6px 9px;border:1px solid #1a1a28;transition:border-color .15s}
.ji:hover{border-color:#252535}
.ji.live-active{border-color:#f9c30033;background:#f9c30006}
.ji-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:4px;gap:4px}
.ji-name{font-size:10px;color:#555;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;flex:1;cursor:default}
.ji-type{font-size:9px;color:#2a2a3a;padding:1px 4px;background:#1a1a28;border-radius:3px;flex-shrink:0}
.ji-val{font-size:11px;color:#13bfd5;font-weight:600;min-width:46px;text-align:right;flex-shrink:0}
.ji-val.live{color:#f9c300}
input[type=range]{width:100%;height:3px;-webkit-appearance:none;appearance:none;background:#1e1e2e;border-radius:2px;outline:none;cursor:pointer}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:11px;height:11px;background:#13bfd5;border-radius:50%;transition:transform .1s}
input[type=range].live-sl::-webkit-slider-thumb{background:#f9c300}
input[type=range]::-webkit-slider-thumb:hover{transform:scale(1.3)}
.timeline-panel{height:190px;flex-shrink:0;background:#0a0a12;border-top:1px solid #13bfd530;display:flex;flex-direction:column}
.tl-controls{height:38px;background:#0d0d18;border-bottom:1px solid #1a1a28;display:flex;align-items:center;padding:0 12px;gap:8px;flex-shrink:0}
.tl-btn{padding:4px 8px;background:#12121e;border:1px solid #1e1e2e;border-radius:5px;color:#666;font-size:11px;cursor:pointer;white-space:nowrap;transition:all .15s}
.tl-btn:hover{border-color:#13bfd544;color:#99d5db}
.tl-btn.on{background:#13bfd510;border-color:#13bfd555;color:#13bfd5}
.tl-btn.accent{background:#13bfd520;border-color:#13bfd5;color:#13bfd5}
.tl-btn.danger{border-color:#ff444444;color:#ff6666}
.tl-btn.danger:hover{background:#ff000010;border-color:#ff4444}
.tl-sep{width:1px;height:18px;background:#1e1e2e;flex-shrink:0}
.tl-label{font-size:10px;color:#444;white-space:nowrap}
.tl-select{background:#12121e;border:1px solid #1e1e2e;border-radius:5px;color:#777;font-size:11px;padding:3px 6px;outline:none;cursor:pointer}
.tl-dur{background:#12121e;border:1px solid #1e1e2e;border-radius:5px;color:#aaa;font-size:11px;padding:3px 6px;outline:none;width:52px;text-align:center}
.tl-kf-count{font-size:10px;color:#333;margin-left:auto;white-space:nowrap}
.tl-time-display{font-size:11px;color:#555;font-family:monospace;min-width:48px;text-align:center}
.tl-body{flex:1;display:flex;overflow:hidden}
.tl-canvas-wrap{flex:1;position:relative;overflow:hidden}
#tlCanvas{display:block;cursor:crosshair}
::-webkit-scrollbar{width:3px}
::-webkit-scrollbar-track{background:transparent}
::-webkit-scrollbar-thumb{background:#1e1e2e;border-radius:2px}
</style>
</head>
<body>
<header>
  <h1>&#x1F916; G1 URDF Viewer</h1>
  <span class="badge" id="statusBadge">로딩 중... (Loading)</span>
  <span class="badge" id="liveBadge" style="display:none">● LIVE</span>
  <span style="margin-left:auto;font-size:10px;color:#2a2a3a" id="imuDisp">IMU: -</span>
</header>

<div class="main">
  <!-- LEFT -->
  <div class="left">
    <div class="sec">
      <div class="sec-title">연결 상태 (Connection)</div>
      <div style="display:flex;align-items:center;gap:8px;margin-bottom:8px">
        <div class="status-dot" id="connDot"></div>
        <span style="font-size:11px;color:#555" id="connText">연결 확인 중...</span>
      </div>
      <button class="tb" id="liveBtn" onclick="toggleLive()">
        <span class="ic">&#x25CF;</span>실시간 모드 (Live Mode)
      </button>
    </div>
    <div class="sec">
      <div class="sec-title">뷰 (View)</div>
      <button class="tb on" id="gridBtn"><span class="ic">&#x22DE;</span>그리드 (Grid)</button>
      <button class="tb" id="wireBtn"><span class="ic">&#x25A6;</span>와이어프레임 (Wireframe)</button>
      <button class="tb" id="axisBtn"><span class="ic">&#x2197;</span>관절 좌표축 (Joint Axes)</button>
      <button class="tb" id="ssBtn"><span class="ic">&#x1F4F7;</span>스크린샷 (Screenshot)</button>
    </div>
    <div class="sec">
      <div class="sec-title">포즈 (Pose)</div>
      <button class="tb" id="resetBtn"><span class="ic">&#x21BA;</span>리셋 (Reset)</button>
      <div class="pose-row">
        <button class="pose-btn" id="savePoseBtn">&#x1F4BE; 저장 (Save)</button>
        <button class="pose-btn" id="loadPoseBtn">&#x1F4C2; 불러오기 (Load)</button>
      </div>
      <input type="file" id="poseFileIn" accept=".json">
    </div>
    <div class="sec" id="infoSec" style="display:none">
      <div class="sec-title">모델 정보 (Info)</div>
      <div id="infoRows"></div>
    </div>
    <div class="sec" id="imuSec" style="display:none">
      <div class="sec-title">IMU (Pelvis)</div>
      <div id="imuRows"></div>
    </div>
  </div>

  <!-- VIEWPORT -->
  <div class="viewport" id="vp">
    <canvas id="cv"></canvas>
    <div class="load-overlay" id="loadOv">
      <div class="load-title">모델 로딩 중... (Loading model...)</div>
      <div class="pbar-bg"><div class="pbar-fill" id="pbar"></div></div>
      <div class="pbar-text" id="ptext">URDF 로딩... (Loading URDF...)</div>
    </div>
    <div class="hud">
      <span id="fpsEl">FPS: --</span>
      <span style="color:#1e1e28">좌클릭 (Rotate) &nbsp; 우클릭 (Pan) &nbsp; 휠 (Zoom)</span>
    </div>
    <div class="tooltip" id="tt"></div>
  </div>

  <!-- RIGHT -->
  <div class="right">
    <div class="right-head">
      <b>관절 제어 (Joints)</b>
      <input class="search-box" id="searchBox" placeholder="검색... (Search)" type="text">
      <span class="jcount" id="jcount">-</span>
    </div>
    <div class="joints" id="jointsEl">
      <div class="no-joint">로딩 중... (Loading...)</div>
    </div>
  </div>
</div>

<!-- TIMELINE -->
<div class="timeline-panel">
  <div class="tl-controls">
    <button class="tl-btn" id="tlToStart">&#x23EE;</button>
    <button class="tl-btn" id="tlPlayPause">&#x25B6;</button>
    <button class="tl-btn" id="tlStop">&#x25A0;</button>
    <span class="tl-time-display" id="tlTimeDisp">0.00s</span>
    <div class="tl-sep"></div>
    <button class="tl-btn accent" id="tlAddKf">+ 키프레임 (Add KF)</button>
    <button class="tl-btn danger" id="tlDelKf">&#x2715; 삭제</button>
    <div class="tl-sep"></div>
    <span class="tl-label">길이 (Duration)</span>
    <input class="tl-dur" id="tlDurInput" type="number" value="5" min="1" max="60" step="0.5">
    <span class="tl-label">s &nbsp; 속도</span>
    <select class="tl-select" id="tlSpeedSel">
      <option value="0.25">0.25×</option>
      <option value="0.5">0.5×</option>
      <option value="1" selected>1×</option>
      <option value="2">2×</option>
    </select>
    <button class="tl-btn" id="tlLoopBtn">&#x1F501; 루프</button>
    <div class="tl-sep"></div>
    <button class="tl-btn" id="tlSaveMotion">&#x1F4BE; 모션 저장</button>
    <button class="tl-btn" id="tlLoadMotion">&#x1F4C2; 불러오기</button>
    <input type="file" id="motionFileIn" accept=".json">
    <span class="tl-kf-count" id="tlKfCount">키프레임 0개</span>
  </div>
  <div class="tl-body">
    <div class="tl-canvas-wrap" id="tlWrap">
      <canvas id="tlCanvas"></canvas>
    </div>
  </div>
</div>

<script>
// ═══════════════════════════════════════
// ORBIT
// ═══════════════════════════════════════
class Orbit {
  constructor(cam,el){
    this.cam=cam;this.el=el;this.target=new THREE.Vector3(0,.8,0);
    this.phi=1.2;this.theta=0.5;this.r=3.5;
    this._dn=false;this._btn=-1;this._lx=0;this._ly=0;
    el.addEventListener('mousedown',e=>{this._dn=true;this._btn=e.button;this._lx=e.clientX;this._ly=e.clientY;});
    window.addEventListener('mousemove',e=>{
      if(!this._dn)return;
      const dx=e.clientX-this._lx,dy=e.clientY-this._ly;this._lx=e.clientX;this._ly=e.clientY;
      if(this._btn===0){this.theta-=dx*.005;this.phi=Math.max(.04,Math.min(Math.PI-.04,this.phi-dy*.005));}
      else if(this._btn===2){const f=this.r*.0012;const rt=new THREE.Vector3().setFromMatrixColumn(cam.matrix,0);const up=new THREE.Vector3().setFromMatrixColumn(cam.matrix,1);this.target.addScaledVector(rt,-dx*f).addScaledVector(up,dy*f);}
      this.update();
    });
    window.addEventListener('mouseup',()=>this._dn=false);
    el.addEventListener('wheel',e=>{e.preventDefault();this.r=Math.max(.15,Math.min(60,this.r*(e.deltaY>0?1.1:.9)));this.update();},{passive:false});
    el.addEventListener('contextmenu',e=>e.preventDefault());
    this.update();
  }
  update(){const s=Math.sin(this.phi);this.cam.position.set(this.target.x+this.r*s*Math.sin(this.theta),this.target.y+this.r*Math.cos(this.phi),this.target.z+this.r*s*Math.cos(this.theta));this.cam.lookAt(this.target);}
  focusBox(box){const c=box.getCenter(new THREE.Vector3());const sz=box.getSize(new THREE.Vector3()).length();this.target.copy(c);this.r=sz*1.3;this.update();}
}

// ═══════════════════════════════════════
// STL PARSER
// ═══════════════════════════════════════
function parseSTL(buf){
  const dv=new DataView(buf);
  if(buf.byteLength<84)return parseASCII(new TextDecoder().decode(buf));
  const n=dv.getUint32(80,true);
  if(Math.abs(buf.byteLength-(84+n*50))<=4)return parseBin(dv,n);
  return parseASCII(new TextDecoder().decode(buf));
}
function parseBin(dv,n){
  const pos=new Float32Array(n*9),nrm=new Float32Array(n*9);let o=84;
  for(let i=0;i<n;i++){
    const nx=dv.getFloat32(o,true),ny=dv.getFloat32(o+4,true),nz=dv.getFloat32(o+8,true);o+=12;
    for(let j=0;j<3;j++){const b=i*9+j*3;pos[b]=dv.getFloat32(o,true);pos[b+1]=dv.getFloat32(o+4,true);pos[b+2]=dv.getFloat32(o+8,true);nrm[b]=nx;nrm[b+1]=ny;nrm[b+2]=nz;o+=12;}
    o+=2;
  }
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(pos,3));g.setAttribute('normal',new THREE.BufferAttribute(nrm,3));return g;
}
function parseASCII(txt){
  const pos=[],nrm=[];let nx=0,ny=0,nz=0;
  for(const ln of txt.split('\n')){const l=ln.trim();
    if(l.startsWith('facet normal')){const m=l.match(/normal\s+([\S]+)\s+([\S]+)\s+([\S]+)/);if(m){nx=+m[1];ny=+m[2];nz=+m[3];}}
    else if(l.startsWith('vertex')){const m=l.match(/vertex\s+([\S]+)\s+([\S]+)\s+([\S]+)/);if(m){pos.push(+m[1],+m[2],+m[3]);nrm.push(nx,ny,nz);}}
  }
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(pos,3));g.setAttribute('normal',new THREE.Float32BufferAttribute(nrm,3));return g;
}

// ═══════════════════════════════════════
// URDF PARSER
// ═══════════════════════════════════════
function parseURDF(xml){
  const doc=new DOMParser().parseFromString(xml,'text/xml');
  const links={},joints={};
  doc.querySelectorAll('link').forEach(el=>{
    const name=el.getAttribute('name');links[name]={name,visuals:[]};
    el.querySelectorAll('visual').forEach(v=>{
      const me=v.querySelector('geometry mesh');if(!me)return;
      const fn=me.getAttribute('filename')||'';
      const sc=(me.getAttribute('scale')||'1 1 1').split(/\s+/).map(Number);
      links[name].visuals.push({fn,sc,origin:parseOrig(v.querySelector('origin'))});
    });
  });
  doc.querySelectorAll('joint').forEach(el=>{
    const name=el.getAttribute('name'),type=el.getAttribute('type')||'fixed';
    const parent=el.querySelector('parent')?.getAttribute('link')||'';
    const child=el.querySelector('child')?.getAttribute('link')||'';
    const axEl=el.querySelector('axis');
    const axis=(axEl?.getAttribute('xyz')||'0 0 1').split(/\s+/).map(Number);
    const lim=el.querySelector('limit');
    joints[name]={name,type,parent,child,origin:parseOrig(el.querySelector('origin')),axis,
      limit:{lower:lim?+lim.getAttribute('lower'):-3.14,upper:lim?+lim.getAttribute('upper'):3.14}};
  });
  return{links,joints};
}
function parseOrig(el){
  if(!el)return{xyz:[0,0,0],rpy:[0,0,0]};
  return{xyz:(el.getAttribute('xyz')||'0 0 0').split(/\s+/).map(Number),rpy:(el.getAttribute('rpy')||'0 0 0').split(/\s+/).map(Number)};
}
const basename=s=>s.split(/[/\\]/).pop().toLowerCase();
const sid=n=>n.replace(/\W/g,'_');

// ═══════════════════════════════════════
// THREE.JS SETUP
// ═══════════════════════════════════════
const cv=document.getElementById('cv'),vp=document.getElementById('vp');
const renderer=new THREE.WebGLRenderer({canvas:cv,antialias:true,preserveDrawingBuffer:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
renderer.setClearColor(0x0a0a0f);
const scene=new THREE.Scene();
const camera=new THREE.PerspectiveCamera(45,1,.001,500);
const orbit=new Orbit(camera,cv);
scene.add(new THREE.AmbientLight(0xffffff,.45));
const dl1=new THREE.DirectionalLight(0xffffff,.8);dl1.position.set(5,10,5);scene.add(dl1);
const dl2=new THREE.DirectionalLight(0x8899ff,.25);dl2.position.set(-5,5,-5);scene.add(dl2);
const grid=new THREE.GridHelper(14,28,0x181828,0x181828);scene.add(grid);
const raycaster=new THREE.Raycaster();const mouse=new THREE.Vector2();
function resize(){const w=vp.clientWidth,h=vp.clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();}
resize();new ResizeObserver(resize).observe(vp);

// ═══════════════════════════════════════
// APP STATE
// ═══════════════════════════════════════
let robotRoot=null,jointObjs={},baseQs={},jointDefs={};
let allMeshes=[],allAxes=[],wireMode=false,axisMode=false;
let selectedMesh=null;
let sliderEls={},valEls={};
let liveMode=false,liveEvt=null;

// ═══════════════════════════════════════
// TIMELINE STATE
// ═══════════════════════════════════════
let keyframes=[],kfIdCounter=0,currentTime=0,duration=5,isPlaying=false,playSpeed=1,loopMode=false;
let selectedKfId=null,playLastT=null,tlDragging=false,tlDragKfId=null,tlSeeking=false;

// ═══════════════════════════════════════
// RENDER LOOP
// ═══════════════════════════════════════
let fps=0,fpsT=performance.now();
function renderLoop(now){
  requestAnimationFrame(renderLoop);
  if(isPlaying&&!liveMode){
    if(playLastT!==null){
      currentTime+=(now-playLastT)/1000*playSpeed;
      if(currentTime>=duration){
        if(loopMode){currentTime=currentTime%duration;}
        else{currentTime=duration;isPlaying=false;playLastT=null;document.getElementById('tlPlayPause').textContent='▶';}
      }
    }
    playLastT=now;
    applyAtTime(currentTime);
  }
  renderer.render(scene,camera);
  drawTimeline();
  document.getElementById('tlTimeDisp').textContent=currentTime.toFixed(2)+'s';
  fps++;if(now-fpsT>=1000){document.getElementById('fpsEl').textContent='FPS: '+fps;fps=0;fpsT=now;}
}
requestAnimationFrame(renderLoop);

// ═══════════════════════════════════════
// LOAD ROBOT FROM SERVER
// ═══════════════════════════════════════
async function loadRobot(){
  document.getElementById('loadOv').style.display='flex';
  const pbar=document.getElementById('pbar'),ptext=document.getElementById('ptext');
  try{
    if(robotRoot){scene.remove(robotRoot);robotRoot=null;}
    jointObjs={};baseQs={};allMeshes=[];allAxes=[];sliderEls={};valEls={};

    // 1. URDF 로드
    ptext.textContent='URDF 로딩... (Loading URDF...)';pbar.style.width='5%';
    const urdfResp=await fetch('/api/urdf');
    const urdfTxt=await urdfResp.text();
    const{links,joints}=parseURDF(urdfTxt);
    jointDefs=joints;

    // 2. STL 목록
    ptext.textContent='STL 목록 확인... (Getting mesh list...)';pbar.style.width='10%';
    const meshListResp=await fetch('/api/meshes');
    const meshListData=await meshListResp.json();
    const serverFiles=new Set(meshListData.files.map(f=>f.toLowerCase()));

    // 3. 필요한 STL 계산
    const needed=new Set();
    for(const l of Object.values(links))
      for(const v of l.visuals){const b=basename(v.fn);if(serverFiles.has(b))needed.add(b);}

    // 4. STL 병렬 로드
    const geos={};const total=needed.size;let loaded=0;
    await Promise.all([...needed].map(async b=>{
      const resp=await fetch('/api/mesh/'+b);
      const buf=await resp.arrayBuffer();
      const geo=parseSTL(buf);
      geo.computeVertexNormals();
      geos[b]=geo;
      loaded++;
      pbar.style.width=`${10+(loaded/total)*85}%`;
      ptext.textContent=`STL ${loaded} / ${total}`;
    }));

    // 5. 씬 빌드
    ptext.textContent='씬 빌드... (Building scene...)';pbar.style.width='98%';
    await new Promise(r=>setTimeout(r,0));

    function getColor(n){const k=n.toLowerCase();
      if(k.includes('left')||k.startsWith('l_'))return 0x4fc3f7;
      if(k.includes('right')||k.startsWith('r_'))return 0xf48fb1;
      if(k.includes('head'))return 0xffb74d;
      if(k.includes('hand')||k.includes('finger')||k.includes('thumb')||k.includes('index')||k.includes('middle')||k.includes('palm'))return 0xce93d8;
      if(k.includes('hip')||k.includes('pelvis')||k.includes('waist')||k.includes('torso'))return 0x90a4ae;
      if(k.includes('foot')||k.includes('ankle')||k.includes('knee'))return 0xa5d6a7;
      return 0x78909c;
    }
    const childSet=new Set(Object.values(joints).map(j=>j.child));
    const rootName=Object.keys(links).find(l=>!childSet.has(l))||Object.keys(links)[0];
    function mkLink(lname){
      const link=links[lname];if(!link)return null;
      const grp=new THREE.Group();grp.name='link:'+lname;
      for(const v of link.visuals){const b=basename(v.fn),geo=geos[b];if(!geo)continue;
        const mat=new THREE.MeshPhongMaterial({color:getColor(lname),specular:0x111122,shininess:28});
        const mesh=new THREE.Mesh(geo,mat);mesh.userData={linkName:lname};
        mesh.position.set(...v.origin.xyz);mesh.setRotationFromEuler(new THREE.Euler(...v.origin.rpy,'XYZ'));mesh.scale.set(...v.sc);
        allMeshes.push(mesh);grp.add(mesh);}
      const ax=new THREE.AxesHelper(.08);ax.visible=false;allAxes.push(ax);grp.add(ax);
      for(const jt of Object.values(joints).filter(j=>j.parent===lname)){
        const jg=new THREE.Group();jg.name='joint:'+jt.name;
        jg.position.set(...jt.origin.xyz);jg.setRotationFromEuler(new THREE.Euler(...jt.origin.rpy,'XYZ'));
        jointObjs[jt.name]=jg;baseQs[jt.name]=jg.quaternion.clone();
        const child=mkLink(jt.child);if(child)jg.add(child);grp.add(jg);}
      return grp;
    }
    robotRoot=mkLink(rootName);
    if(robotRoot){
      robotRoot.rotation.x=-Math.PI/2;
      scene.add(robotRoot);
      const box=new THREE.Box3().setFromObject(robotRoot);
      robotRoot.position.y=-box.min.y;
      orbit.focusBox(new THREE.Box3().setFromObject(robotRoot));
    }
    buildSliders(joints);
    const mv=Object.values(joints).filter(j=>j.type!=='fixed').length;
    document.getElementById('infoSec').style.display='block';
    document.getElementById('infoRows').innerHTML=
      [['링크',Object.keys(links).length],['관절',Object.keys(joints).length],['가동',mv],['메시',allMeshes.length]]
      .map(([k,v])=>`<div class="info-row"><span>${k}</span><span class="info-val">${v}</span></div>`).join('');
    const b=document.getElementById('statusBadge');b.textContent='로드 완료 (Ready)';b.className='badge ok';
    pbar.style.width='100%';

    // 연결 상태 확인
    checkConnection();

  }catch(e){console.error(e);document.getElementById('statusBadge').textContent='오류: '+e.message;document.getElementById('statusBadge').className='badge err';}
  document.getElementById('loadOv').style.display='none';
}

async function checkConnection(){
  try{
    const r=await fetch('/api/status');const d=await r.json();
    const dot=document.getElementById('connDot');const txt=document.getElementById('connText');
    if(d.connected){
      dot.className='status-dot on';txt.textContent='로봇 연결됨 (Connected)';txt.style.color='#4caf80';
    } else {
      dot.className='status-dot err';txt.textContent='시뮬레이션 모드';txt.style.color='#ff6666';
    }
  }catch(e){console.error(e);}
}

// ═══════════════════════════════════════
// LIVE MODE
// ═══════════════════════════════════════
function toggleLive(){
  liveMode=!liveMode;
  const btn=document.getElementById('liveBtn');
  const badge=document.getElementById('liveBadge');
  if(liveMode){
    btn.className='tb live-on';btn.innerHTML='<span class="ic">&#x23F9;</span>실시간 중지 (Stop Live)';
    badge.style.display='';badge.className='badge live';
    isPlaying=false;document.getElementById('tlPlayPause').textContent='▶';
    startSSE();
    document.querySelectorAll('.ji-val').forEach(el=>el.classList.add('live'));
    document.querySelectorAll('.jslider').forEach(sl=>sl.classList.add('live-sl'));
    document.querySelectorAll('.ji').forEach(el=>el.classList.add('live-active'));
  } else {
    btn.className='tb';btn.innerHTML='<span class="ic">&#x25CF;</span>실시간 모드 (Live Mode)';
    badge.style.display='none';
    stopSSE();
    document.querySelectorAll('.ji-val').forEach(el=>el.classList.remove('live'));
    document.querySelectorAll('.jslider').forEach(sl=>sl.classList.remove('live-sl'));
    document.querySelectorAll('.ji').forEach(el=>el.classList.remove('live-active'));
  }
}

function startSSE(){
  if(liveEvt)liveEvt.close();
  liveEvt=new EventSource('/api/joint_states');
  liveEvt.onmessage=e=>{
    if(!liveMode)return;
    const data=JSON.parse(e.data);
    // IMU 표시
    if(data._imu){
      const [r,p,y]=data._imu;
      document.getElementById('imuDisp').textContent=
        `IMU  R:${(r*180/Math.PI).toFixed(1)}°  P:${(p*180/Math.PI).toFixed(1)}°  Y:${(y*180/Math.PI).toFixed(1)}°`;
      document.getElementById('imuSec').style.display='block';
      document.getElementById('imuRows').innerHTML=
        [['Roll',(r*180/Math.PI).toFixed(2)+'°'],['Pitch',(p*180/Math.PI).toFixed(2)+'°'],['Yaw',(y*180/Math.PI).toFixed(2)+'°']]
        .map(([k,v])=>`<div class="info-row imu-row"><span>${k}</span><span class="info-val">${v}</span></div>`).join('');
    }
    // 관절 적용
    delete data._imu;delete data._connected;
    applyPose(data);
  };
  liveEvt.onerror=()=>{if(liveMode)setTimeout(startSSE,1000);};
}

function stopSSE(){
  if(liveEvt){liveEvt.close();liveEvt=null;}
  document.getElementById('imuDisp').textContent='IMU: -';
}

// ═══════════════════════════════════════
// JOINT SLIDERS
// ═══════════════════════════════════════
function buildSliders(joints){
  const movable=Object.values(joints).filter(j=>j.type!=='fixed');
  document.getElementById('jcount').textContent=movable.length+'개';
  const groups={'머리':[],'허리/골반':[],'왼팔':[],'오른팔':[],'왼손':[],'오른손':[],'왼다리':[],'오른다리':[],'기타':[]};
  for(const j of movable){const n=j.name.toLowerCase();
    if(n.includes('head'))groups['머리'].push(j);
    else if(n.includes('waist'))groups['허리/골반'].push(j);
    else if((n.includes('left'))&&(n.includes('shoulder')||n.includes('elbow')||n.includes('wrist')))groups['왼팔'].push(j);
    else if((n.includes('right'))&&(n.includes('shoulder')||n.includes('elbow')||n.includes('wrist')))groups['오른팔'].push(j);
    else if((n.includes('left'))&&(n.includes('hand')||n.includes('finger')||n.includes('thumb')||n.includes('index')||n.includes('middle')||n.includes('palm')))groups['왼손'].push(j);
    else if((n.includes('right'))&&(n.includes('hand')||n.includes('finger')||n.includes('thumb')||n.includes('index')||n.includes('middle')||n.includes('palm')))groups['오른손'].push(j);
    else if((n.includes('left'))&&(n.includes('hip')||n.includes('knee')||n.includes('ankle')))groups['왼다리'].push(j);
    else if((n.includes('right'))&&(n.includes('hip')||n.includes('knee')||n.includes('ankle')))groups['오른다리'].push(j);
    else groups['기타'].push(j);
  }
  const el=document.getElementById('jointsEl');
  if(!movable.length){el.innerHTML='<div class="no-joint">가동 관절 없음</div>';return;}
  let html='';
  for(const[gname,jlist]of Object.entries(groups)){
    if(!jlist.length)continue;const gid=sid(gname);
    html+=`<div class="group-header" onclick="toggleG('${gid}')"><span class="garr open" id="arr_${gid}">&#x25B6;</span><span>${gname}</span><span style="color:#2a2a3a;margin-left:4px">(${jlist.length})</span></div><div class="group-body" id="gb_${gid}">`;
    for(const j of jlist){html+=`<div class="ji" id="ji_${sid(j.name)}" data-joint="${j.name}"><div class="ji-head"><span class="ji-name" title="${j.name}">${j.name}</span><span class="ji-type">${j.type}</span><span class="ji-val" id="v_${sid(j.name)}">0.0°</span></div><input type="range" class="jslider" data-joint="${j.name}" min="${j.limit.lower.toFixed(4)}" max="${j.limit.upper.toFixed(4)}" step="0.001" value="0" id="s_${sid(j.name)}"></div>`;}
    html+='</div>';
  }
  el.innerHTML=html;
  el.querySelectorAll('.jslider').forEach(sl=>{
    const jname=sl.dataset.joint;sliderEls[jname]=sl;valEls[jname]=document.getElementById('v_'+sid(jname));
    sl.addEventListener('input',()=>{
      if(liveMode)return; // 라이브 모드에서는 수동 조작 무시
      const a=parseFloat(sl.value);setAngle(jname,a);valEls[jname].textContent=(a*180/Math.PI).toFixed(1)+'°';
    });
  });
  setTimeout(()=>document.querySelectorAll('.group-body').forEach(b=>b.style.maxHeight=b.scrollHeight+'px'),60);
}

window.toggleG=function(gid){
  const body=document.getElementById('gb_'+gid),arr=document.getElementById('arr_'+gid);
  const open=arr.classList.contains('open');
  body.style.maxHeight=open?'0px':body.scrollHeight+'px';arr.classList.toggle('open',!open);
};
document.getElementById('searchBox').addEventListener('input',function(){
  const q=this.value.toLowerCase().trim();
  document.querySelectorAll('.ji').forEach(el=>el.style.display=(!q||el.dataset.joint?.toLowerCase().includes(q))?'':'none');
});

function setAngle(jname,angle){
  const jobj=jointObjs[jname],jdef=jointDefs[jname];if(!jobj||!jdef)return;
  const ax=new THREE.Vector3(...jdef.axis).normalize();
  jobj.quaternion.copy(baseQs[jname]).multiply(new THREE.Quaternion().setFromAxisAngle(ax,angle));
}

// ═══════════════════════════════════════
// TIMELINE
// ═══════════════════════════════════════
function getCurrentPose(){
  const j={};for(const[jname,sl]of Object.entries(sliderEls))j[jname]=parseFloat(sl.value);return j;
}
function applyPose(joints){
  for(const[jname,angle]of Object.entries(joints)){
    setAngle(jname,angle);
    if(sliderEls[jname]){sliderEls[jname].value=angle;}
    if(valEls[jname])valEls[jname].textContent=(angle*180/Math.PI).toFixed(1)+'°';
  }
}
function applyAtTime(t){
  if(!keyframes.length)return;
  const sorted=[...keyframes].sort((a,b)=>a.time-b.time);
  if(sorted.length===1){applyPose(sorted[0].joints);return;}
  if(t<=sorted[0].time){applyPose(sorted[0].joints);return;}
  if(t>=sorted[sorted.length-1].time){applyPose(sorted[sorted.length-1].joints);return;}
  let prev=sorted[0],next=sorted[1];
  for(let i=0;i<sorted.length-1;i++){if(sorted[i].time<=t&&sorted[i+1].time>=t){prev=sorted[i];next=sorted[i+1];break;}}
  const alpha=next.time===prev.time?1:(t-prev.time)/(next.time-prev.time);
  const ease=alpha*alpha*(3-2*alpha);
  const merged={};
  const allJ=new Set([...Object.keys(prev.joints),...Object.keys(next.joints)]);
  for(const jn of allJ){merged[jn]=(prev.joints[jn]??0)+((next.joints[jn]??0)-(prev.joints[jn]??0))*ease;}
  applyPose(merged);
}
function addKeyframe(){
  if(!Object.keys(sliderEls).length){alert('먼저 로봇이 로드되어야 합니다');return;}
  const SNAP=0.05;
  const existing=keyframes.find(k=>Math.abs(k.time-currentTime)<SNAP);
  if(existing){existing.joints=getCurrentPose();selectedKfId=existing.id;}
  else{const kf={id:kfIdCounter++,time:parseFloat(currentTime.toFixed(3)),joints:getCurrentPose()};keyframes.push(kf);selectedKfId=kf.id;}
  updateKfCount();
}
function deleteSelectedKf(){if(selectedKfId===null)return;keyframes=keyframes.filter(k=>k.id!==selectedKfId);selectedKfId=null;updateKfCount();}
function updateKfCount(){document.getElementById('tlKfCount').textContent=`키프레임 ${keyframes.length}개`;}

const tlCanvas=document.getElementById('tlCanvas'),tlWrap=document.getElementById('tlWrap');
function resizeTlCanvas(){tlCanvas.width=tlWrap.clientWidth;tlCanvas.height=tlWrap.clientHeight;}
resizeTlCanvas();new ResizeObserver(resizeTlCanvas).observe(tlWrap);
function timeToX(t){return Math.round((t/duration)*tlCanvas.width);}
function xToTime(x){return Math.max(0,Math.min(duration,(x/tlCanvas.width)*duration));}
function drawTimeline(){
  const canvas=tlCanvas,ctx=canvas.getContext('2d');
  const W=canvas.width,H=canvas.height;if(W<10||H<10)return;
  const TRACK_H=H-20;
  ctx.fillStyle='#0a0a12';ctx.fillRect(0,0,W,H);
  const step=duration<=10?0.5:duration<=30?1:2;
  for(let t=0;t<=duration;t+=step){
    const x=timeToX(t);const isWhole=Math.abs(t%1)<.01;
    ctx.strokeStyle=isWhole?'#1e1e2e':'#161620';ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,TRACK_H);ctx.stroke();
    if(isWhole||duration<=5){ctx.fillStyle='#2a2a3a';ctx.font='9px monospace';ctx.fillText(t.toFixed(1)+'s',x+2,H-5);}
  }
  const sorted=[...keyframes].sort((a,b)=>a.time-b.time);
  if(sorted.length>=2){
    ctx.strokeStyle='#13bfd520';ctx.lineWidth=1.5;
    ctx.beginPath();ctx.moveTo(timeToX(sorted[0].time),TRACK_H/2);
    for(const kf of sorted.slice(1))ctx.lineTo(timeToX(kf.time),TRACK_H/2);
    ctx.stroke();
  }
  for(const kf of sorted){
    const x=timeToX(kf.time);const sel=kf.id===selectedKfId;const R=sel?9:7;
    ctx.shadowColor=sel?'#f9c300':'#13bfd5';ctx.shadowBlur=sel?10:4;
    ctx.fillStyle=sel?'#f9c300':'#13bfd5';
    ctx.beginPath();ctx.moveTo(x,TRACK_H/2-R);ctx.lineTo(x+R,TRACK_H/2);ctx.lineTo(x,TRACK_H/2+R);ctx.lineTo(x-R,TRACK_H/2);ctx.closePath();ctx.fill();
    ctx.shadowBlur=0;
    ctx.fillStyle=sel?'#f9c300':'#13bfd566';ctx.font='9px monospace';ctx.fillText(kf.time.toFixed(2),x+4,TRACK_H/2-12);
  }
  const px=timeToX(currentTime);
  ctx.shadowColor='#ff4444';ctx.shadowBlur=6;ctx.strokeStyle='#ff5555';ctx.lineWidth=2;
  ctx.beginPath();ctx.moveTo(px,0);ctx.lineTo(px,TRACK_H);ctx.stroke();
  ctx.shadowBlur=0;ctx.fillStyle='#ff5555';
  ctx.beginPath();ctx.moveTo(px-6,0);ctx.lineTo(px+6,0);ctx.lineTo(px,10);ctx.closePath();ctx.fill();
}
function getKfAtX(x){return keyframes.find(kf=>Math.abs(timeToX(kf.time)-x)<10)||null;}
tlCanvas.addEventListener('mousedown',e=>{
  const rect=tlCanvas.getBoundingClientRect();const x=e.clientX-rect.left;
  const kf=getKfAtX(x);
  if(kf){selectedKfId=kf.id;tlDragging=true;tlDragKfId=kf.id;currentTime=kf.time;if(!liveMode)applyAtTime(kf.time);}
  else{tlSeeking=true;currentTime=xToTime(x);if(!isPlaying&&!liveMode)applyAtTime(currentTime);}
});
window.addEventListener('mousemove',e=>{
  if(tlDragging&&tlDragKfId!==null){
    const rect=tlCanvas.getBoundingClientRect();const kf=keyframes.find(k=>k.id===tlDragKfId);
    if(kf){kf.time=parseFloat(xToTime(e.clientX-rect.left).toFixed(3));currentTime=kf.time;if(!liveMode)applyAtTime(kf.time);}
  } else if(tlSeeking){
    const rect=tlCanvas.getBoundingClientRect();currentTime=xToTime(e.clientX-rect.left);
    if(!isPlaying&&!liveMode)applyAtTime(currentTime);
  }
});
window.addEventListener('mouseup',()=>{tlDragging=false;tlDragKfId=null;tlSeeking=false;});
tlCanvas.addEventListener('dblclick',e=>{
  const rect=tlCanvas.getBoundingClientRect();currentTime=xToTime(e.clientX-rect.left);addKeyframe();
});
document.getElementById('tlToStart').addEventListener('click',()=>{currentTime=0;isPlaying=false;playLastT=null;document.getElementById('tlPlayPause').textContent='▶';if(!liveMode)applyAtTime(0);});
document.getElementById('tlPlayPause').addEventListener('click',function(){
  if(liveMode){alert('라이브 모드 중에는 재생 불가합니다');return;}
  if(!keyframes.length){alert('키프레임을 먼저 추가하세요');return;}
  isPlaying=!isPlaying;this.textContent=isPlaying?'⏸':'▶';
  if(isPlaying){playLastT=null;if(currentTime>=duration)currentTime=0;}
});
document.getElementById('tlStop').addEventListener('click',()=>{isPlaying=false;playLastT=null;currentTime=0;document.getElementById('tlPlayPause').textContent='▶';if(!liveMode)applyAtTime(0);});
document.getElementById('tlAddKf').addEventListener('click',addKeyframe);
document.getElementById('tlDelKf').addEventListener('click',deleteSelectedKf);
document.getElementById('tlDurInput').addEventListener('change',function(){duration=Math.max(1,parseFloat(this.value)||5);this.value=duration;if(currentTime>duration)currentTime=duration;});
document.getElementById('tlSpeedSel').addEventListener('change',function(){playSpeed=parseFloat(this.value);});
document.getElementById('tlLoopBtn').addEventListener('click',function(){loopMode=!loopMode;this.classList.toggle('on',loopMode);});
document.getElementById('tlSaveMotion').addEventListener('click',()=>{
  const a=document.createElement('a');a.download='motion.json';
  a.href=URL.createObjectURL(new Blob([JSON.stringify({duration,keyframes},null,2)],{type:'application/json'}));a.click();
});
document.getElementById('tlLoadMotion').addEventListener('click',()=>document.getElementById('motionFileIn').click());
document.getElementById('motionFileIn').addEventListener('change',async e=>{
  const f=e.target.files[0];if(!f)return;
  const data=JSON.parse(await readText(f));
  duration=data.duration||5;document.getElementById('tlDurInput').value=duration;
  keyframes=data.keyframes||[];kfIdCounter=keyframes.reduce((m,k)=>Math.max(m,k.id+1),0);
  updateKfCount();currentTime=0;if(!liveMode)applyAtTime(0);e.target.value='';
});

// ═══════════════════════════════════════
// OTHER TOOLS
// ═══════════════════════════════════════
document.getElementById('gridBtn').addEventListener('click',function(){grid.visible=!grid.visible;this.classList.toggle('on',grid.visible);});
document.getElementById('wireBtn').addEventListener('click',function(){wireMode=!wireMode;this.classList.toggle('on',wireMode);allMeshes.forEach(m=>m.material.wireframe=wireMode);});
document.getElementById('axisBtn').addEventListener('click',function(){axisMode=!axisMode;this.classList.toggle('on',axisMode);allAxes.forEach(a=>a.visible=axisMode);});
document.getElementById('ssBtn').addEventListener('click',()=>{renderer.render(scene,camera);const a=document.createElement('a');a.download='g1_viewer.png';a.href=cv.toDataURL('image/png');a.click();});
document.getElementById('resetBtn').addEventListener('click',()=>{
  if(liveMode){toggleLive();}
  isPlaying=false;document.getElementById('tlPlayPause').textContent='▶';
  document.querySelectorAll('.jslider').forEach(sl=>{sl.value=0;sl.dispatchEvent(new Event('input'));});
});
document.getElementById('savePoseBtn').addEventListener('click',()=>{
  const pose={};document.querySelectorAll('.jslider').forEach(sl=>{pose[sl.dataset.joint]=parseFloat(sl.value);});
  const a=document.createElement('a');a.download='pose.json';a.href=URL.createObjectURL(new Blob([JSON.stringify(pose,null,2)],{type:'application/json'}));a.click();
});
document.getElementById('loadPoseBtn').addEventListener('click',()=>document.getElementById('poseFileIn').click());
document.getElementById('poseFileIn').addEventListener('change',async e=>{
  const f=e.target.files[0];if(!f)return;
  const pose=JSON.parse(await readText(f));
  Object.entries(pose).forEach(([name,val])=>{const sl=sliderEls[name];if(!sl)return;sl.value=val;sl.dispatchEvent(new Event('input'));});
  e.target.value='';
});
cv.addEventListener('click',e=>{
  if(!allMeshes.length)return;
  const rect=cv.getBoundingClientRect();
  mouse.x=((e.clientX-rect.left)/rect.width)*2-1;
  mouse.y=-((e.clientY-rect.top)/rect.height)*2+1;
  raycaster.setFromCamera(mouse,camera);
  const hits=raycaster.intersectObjects(allMeshes);
  if(selectedMesh){selectedMesh.material.color.set(selectedMesh.userData.origColor||0x78909c);selectedMesh=null;}
  document.querySelectorAll('.ji').forEach(el=>el.classList.remove('highlight'));
  if(!hits.length)return;
  selectedMesh=hits[0].object;
  selectedMesh.userData.origColor=selectedMesh.material.color.getHex();
  selectedMesh.material.color.set(0xf9c300);
  const lname=selectedMesh.userData.linkName;
  Object.values(jointDefs).forEach(j=>{
    if(j.child===lname||j.parent===lname){const el=document.getElementById('ji_'+sid(j.name));if(el){el.scrollIntoView({block:'nearest',behavior:'smooth'});}}
  });
  const tt=document.getElementById('tt');tt.textContent=lname;tt.style.display='block';
  tt.style.left=(e.clientX-rect.left+10)+'px';tt.style.top=(e.clientY-rect.top+8)+'px';
  setTimeout(()=>tt.style.display='none',2000);
});

function readText(f){return new Promise((r,j)=>{const fr=new FileReader();fr.onload=e=>r(e.target.result);fr.onerror=j;fr.readAsText(f);});}

// ═══════════════════════════════════════
// AUTO LOAD ON START
// ═══════════════════════════════════════
window.addEventListener('load',()=>loadRobot());
</script>
</body>
</html>"""

@app.get('/')
def index():
    return HTMLResponse(HTML_PAGE)

if __name__ == '__main__':
    uvicorn.run(app, host='0.0.0.0', port=5001)
