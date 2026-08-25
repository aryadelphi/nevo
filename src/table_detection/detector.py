#!/usr/bin/env python3


import numpy as np
import cv2
import onnxruntime as ort

# ⚠️ ASUMSI — urutan HARUS dicocokkan ke data.yaml training asli
CLASS_NAMES = ['meja_bersih', 'meja_kotor', 'meja_terisi']

INPUT_SIZE     = 640     # ⚠️ ASUMSI — cek ukuran training asli
CONF_THRESHOLD = 0.20    # [DITURUNKAN] Agar tidak memblokir threshold 0.20 di main2.py
IOU_THRESHOLD  = 0.45    # ⚠️ ASUMSI


class Detector:
    def __init__(self, model_path: str):
        # --- OPTIMASI TURBO UNTUK RASPBERRY PI 4 ---
        sess_options = ort.SessionOptions()
        sess_options.intra_op_num_threads = 4  # Gunakan 4 core CPU maksimal
        sess_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        
        self.session = ort.InferenceSession(
            model_path, 
            sess_options=sess_options,
            providers=['CPUExecutionProvider']
        )
        self.input_name = self.session.get_inputs()[0].name

    def _preprocess(self, frame):
        h0, w0 = frame.shape[:2]
        img = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))       # HWC -> CHW
        img = np.expand_dims(img, axis=0)        # add batch dim
        scale_x = w0 / INPUT_SIZE
        scale_y = h0 / INPUT_SIZE
        return img, scale_x, scale_y

    def detect(self, frame):
        """Jalankan satu kali inferensi. Return list of dict (lihat format
        di docstring modul ini). List kosong kalau tidak ada objek terdeteksi."""
        if frame is None:
            return []

        img, scale_x, scale_y = self._preprocess(frame)

        raw = self.session.run(None, {self.input_name: img})[0]
        # Output Ultralytics ONNX standar (tanpa NMS bawaan): shape (1, 4+num_classes, N)
        raw = np.squeeze(raw)          # -> (4+num_classes, N)
        raw = raw.transpose()          # -> (N, 4+num_classes)

        boxes, scores, class_ids = [], [], []
        for row in raw:
            cx_n, cy_n, w_n, h_n = row[:4]
            class_scores = row[4:]
            cls_id = int(np.argmax(class_scores))
            conf = float(class_scores[cls_id])
            if conf < CONF_THRESHOLD:
                continue

            # koordinat masih dalam skala INPUT_SIZE, kembalikan ke skala frame asli
            x1 = (cx_n - w_n / 2) * scale_x
            y1 = (cy_n - h_n / 2) * scale_y
            x2 = (cx_n + w_n / 2) * scale_x
            y2 = (cy_n + h_n / 2) * scale_y

            boxes.append([x1, y1, x2 - x1, y2 - y1])  # format cv2.dnn.NMSBoxes: x,y,w,h
            scores.append(conf)
            class_ids.append(cls_id)

        if not boxes:
            return []

        indices = cv2.dnn.NMSBoxes(boxes, scores, CONF_THRESHOLD, IOU_THRESHOLD)
        if len(indices) == 0:
            return []
        indices = np.array(indices).flatten()

        results = []
        for i in indices:
            x, y, w, h = boxes[i]
            x1, y1, x2, y2 = int(x), int(y), int(x + w), int(y + h)
            cls_id = class_ids[i]
            label = CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else f'class_{cls_id}'
            results.append({
                'label':      label,
                'confidence': float(scores[i]),
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                'cx': int((x1 + x2) / 2),
                'cy': int((y1 + y2) / 2),
            })
        return results