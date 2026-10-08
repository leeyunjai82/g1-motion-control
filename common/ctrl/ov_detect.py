"""
ov_detect.py — OpenVINO Open Model Zoo SSD 검출기 (얼굴 / 사람)

  모델 (common/models/omz, Intel Open Model Zoo 2023.0 FP16, Apache-2.0)
    face   : face-detection-retail-0004    입력 1×3×300×300 BGR
    person : person-detection-0201          입력 1×3×384×384 BGR (retail-0013 은 위에서 내려보는 매장 카메라용 — 눈높이 사람을 놓침)
  출력 1×1×200×7 = [image_id, label, conf, x_min, y_min, x_max, y_max] (0–1), image_id −1 이후는 무효

  det = OvSSD("face", device="CPU", conf=0.6)
  boxes = det(bgr)        # [(x0, y0, x1, y1, conf), ...] 원본 픽셀, conf 내림차순
"""
import os

import cv2
import numpy as np

try:
    import openvino as ov                     # 2023.1 이후
    _Core = ov.Core
except (ImportError, AttributeError):
    from openvino.runtime import Core as _Core   # 2022.x

MODEL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models", "omz")
MODELS = {"face": "face-detection-retail-0004", "person": "person-detection-0201"}


class OvSSD:
    def __init__(self, name, device="CPU", conf=0.5):
        xml = name if name.endswith(".xml") else os.path.join(MODEL_DIR, MODELS.get(name, name) + ".xml")
        if not os.path.exists(xml):
            raise FileNotFoundError(f"모델 없음: {xml}")
        core = _Core()
        self.compiled = core.compile_model(core.read_model(xml), device)
        self.out = self.compiled.output(0)
        _, _, self.h, self.w = [int(d) for d in self.compiled.input(0).shape]
        self.conf = float(conf)
        self.name = os.path.splitext(os.path.basename(xml))[0]
        self.device = device

    def __call__(self, bgr):
        H, W = bgr.shape[:2]
        blob = cv2.resize(bgr, (self.w, self.h)).transpose(2, 0, 1)[None].astype(np.float32)
        det = np.asarray(self.compiled([blob])[self.out]).reshape(-1, 7)
        end = np.where(det[:, 0] < 0)[0]
        if len(end):
            det = det[:end[0]]
        det = det[det[:, 2] >= self.conf]
        out = []
        for _, _, c, x0, y0, x1, y1 in det[np.argsort(-det[:, 2])]:
            x0, x1 = np.clip([x0, x1], 0, 1) * W
            y0, y1 = np.clip([y0, y1], 0, 1) * H
            if x1 - x0 >= 2 and y1 - y0 >= 2:
                out.append((float(x0), float(y0), float(x1), float(y1), float(c)))
        return out
