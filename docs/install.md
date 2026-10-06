# 방화벽 메모 (Ubuntu 24.04)

RedHat 판의 `sudo systemctl stop firewalld` 에 해당하는 내용입니다.
Ubuntu 기본 방화벽 `ufw` 는 설치 직후 비활성입니다. 켜져 있을 때만 로봇 대역을 허용하세요.

```bash
sudo ufw status
sudo ufw allow from 192.168.123.0/24     # 로봇 LAN (DDS)
sudo ufw allow 80/tcp                    # launcher (다른 PC 에서 접속할 때)
sudo ufw allow 50000:50050/tcp           # 웹 UI / 서버
sudo ufw allow 8000/tcp                  # simulator
```

전체 설치 절차는 [`../INSTALL.md`](../INSTALL.md) 참고.
