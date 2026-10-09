#!/usr/bin/env python3
"""
robot_services.py — 로봇 서비스 목록 보기 / 켜기·끄기 (앱의 서비스 on/off 와 같은 기능을 PC 에서)

  SDK 814556d 에는 go2·b2 용 RobotStateClient 만 있음 (RPC 서비스 'robot_state', API 1003 ServiceList · 1001 ServiceSwitch).
  H2 가 같은 서비스로 video_hub · Stereo patch PC1 을 켜고 끄는지는 문서에 없음 → 확인 필요.
  먼저 목록(읽기 전용)으로 응답 여부와 서비스 이름을 확인할 것.

  python utils/robot_services.py                         # 목록 (읽기 전용, 명령 없음)
  python utils/robot_services.py --off video_hub         # 끄기 (yes 확인)
  python utils/robot_services.py --on <서비스 이름>       # 켜기 (이름은 목록에 나온 그대로)
  python utils/robot_services.py --off video_hub --on <이름> --yes    # 확인 없이 (스크립트용)
  python utils/robot_services.py --iface <인터페이스>     # DDS 네트워크 인터페이스 지정 (기본 자동)

  status 는 로봇이 준 원래 값 그대로 출력 (뜻은 문서에 없음 — 앱 화면과 비교해서 확인).
  protect True 인 서비스는 바꾸지 않음 (로봇도 거부, 코드 5202).
  보행·균형 관련 서비스를 끄면 로봇이 넘어질 수 있음 → 이름을 정확히 확인하고 쓸 것.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "common"))
import robot_env   # noqa: E402  H2 SDK 경로(third_party) 를 먼저 쓰게 함

from unitree_sdk2py.go2.robot_state.robot_state_api import (   # noqa: E402
    ROBOT_STATE_ERR_SERVICE_PROTECTED, ROBOT_STATE_ERR_SERVICE_SWITCH)
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient   # noqa: E402


def service_list(rsc):
    code, lst = rsc.ServiceList()
    if code != 0 or lst is None:
        print(f"[services] ❌ ServiceList 실패 (code {code})")
        return None
    return {s.name: s for s in lst}


def show(svcs):
    w = max([len(n) for n in svcs] + [4])
    print(f"  {'name':{w}s}  status  protect")
    for n in sorted(svcs):
        s = svcs[n]
        print(f"  {n:{w}s}  {s.status!s:6s}  {s.protect}")


def main():
    ap = argparse.ArgumentParser(description="로봇 서비스 목록 / 켜기·끄기 (robot_state RPC)")
    ap.add_argument("--on", action="append", default=[], metavar="NAME", help="켤 서비스 (여러 번 가능)")
    ap.add_argument("--off", action="append", default=[], metavar="NAME", help="끌 서비스 (여러 번 가능)")
    ap.add_argument("--yes", action="store_true", help="확인 묻지 않음")
    ap.add_argument("--iface", default=None, help="DDS 네트워크 인터페이스 (예: enp3s0)")
    ap.add_argument("--timeout", type=float, default=3.0)
    a = ap.parse_args()

    robot_env.dds_init(a.iface)
    rsc = RobotStateClient()
    rsc.SetTimeout(a.timeout)
    rsc.Init()

    code, ver = rsc.GetServerApiVersion()
    if code != 0:
        print(f"[services] ❌ robot_state 서비스 응답 없음 (code {code}) — H2 에 이 RPC 가 없거나 네트워크/인터페이스 문제")
        print("           확인: ping 192.168.123.161 · python utils/check_robot_id.py (rt/lowstate 수신)")
        sys.exit(1)
    print(f"[services] robot_state API {ver}")

    svcs = service_list(rsc)
    if svcs is None:
        sys.exit(1)
    print(f"[services] {len(svcs)} 개")
    show(svcs)

    todo = [(n, False) for n in a.off] + [(n, True) for n in a.on]
    if not todo:
        return

    for n, _ in todo:
        if n not in svcs:
            print(f"[services] ❌ '{n}' 없음 — 위 목록의 이름을 그대로 쓸 것")
            sys.exit(2)
        if svcs[n].protect:
            print(f"[services] ❌ '{n}' 은 protect — 바꾸지 않음")
            sys.exit(2)
    plan = ", ".join(f"{n} {'켜기' if on else '끄기'}" for n, on in todo)
    if not a.yes:
        if input(f"[services] {plan} — 진행하려면 yes: ").strip().lower() != "yes":
            print("[services] 취소")
            return

    fail = 0
    for n, on in todo:
        r = rsc.ServiceSwitch(n, on)
        msg = {0: "OK", ROBOT_STATE_ERR_SERVICE_PROTECTED: "protect 로 거부",
               ROBOT_STATE_ERR_SERVICE_SWITCH: "전환 실패"}.get(r, f"code {r}")
        print(f"[services] {n} {'켜기' if on else '끄기'} → {msg}")
        fail += r != 0

    svcs = service_list(rsc)
    if svcs is not None:
        print("[services] 바꾼 뒤")
        show({n: svcs[n] for n, _ in todo if n in svcs})
    sys.exit(1 if fail else 0)


if __name__ == "__main__":
    main()
