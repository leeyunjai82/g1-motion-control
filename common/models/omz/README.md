# OpenVINO Open Model Zoo 검출 모델 (머리 추종용)

| 파일 | 용도 | 입력 | 출처 |
| --- | --- | --- | --- |
| `face-detection-retail-0004.xml/.bin` | 얼굴 | 1×3×300×300 BGR | Intel Open Model Zoo 2023.0, FP16 |
| `person-detection-0201.xml/.bin` | 사람 | 1×3×384×384 BGR | Intel Open Model Zoo 2023.0, FP16 |

- 출력: 1×1×200×7 = `[image_id, label, conf, x_min, y_min, x_max, y_max]` (0–1)
- 받은 곳: `https://storage.openvinotoolkit.org/repositories/open_model_zoo/2023.0/models_bin/1/<이름>/FP16/<이름>.{xml,bin}` (2026-10-08)
- 라이선스: Apache-2.0 (Open Model Zoo)
- 사람 모델 고른 이유: `person-detection-retail-0013` 은 매장 천장 카메라용이라 눈높이 시험 사진에서 가운데 사람을 놓침 (0.05 미만),
  `person-detection-0201` 은 같은 사진 0.81, 절반 크기에서도 0.81. `person-detection-0200`(256) 은 절반 크기에서 놓침
- 쓰는 곳: `common/ctrl/ov_detect.py` → `common/head_track.py`
