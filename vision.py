# vision.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Dict, Tuple, List
import time, cv2, numpy as np # type: ignore


@dataclass
class VisionConfig:
    camera_index: int = 0
    camera_backend: int = cv2.CAP_DSHOW if hasattr(cv2, "CAP_DSHOW") else 0
    work_width_mm: float = 300.0
    work_height_mm: float = 200.0
    required_marker_ids: Tuple[int, int, int, int] = (0, 1, 2, 3)
    min_contour_area: float = 400.0
    homography_hold_sec: float = 0.8
    show_mask: bool = False


@dataclass
class TargetObservation:
    color_name: str
    pixel_x: int
    pixel_y: int
    area: float
    world_x: Optional[float]
    world_y: Optional[float]
    workspace_ready: bool


# (lower_bounds, upper_bounds, BGR draw color)
# 紅色需要兩段 hue，用 list 包兩組
COLOR_TABLE: Dict[str, dict] = {
    "RED": {
        "ranges": [
            (np.array([0, 120, 70], np.uint8),   np.array([10, 255, 255], np.uint8)),
            (np.array([170, 120, 70], np.uint8),  np.array([180, 255, 255], np.uint8)),
        ],
        "bgr": (0, 0, 255),
    },
    "GREEN": {
        "ranges": [
            (np.array([40, 50, 50], np.uint8), np.array([80, 255, 255], np.uint8)),
        ],
        "bgr": (0, 255, 0),
    },
    "BLUE": {
        "ranges": [
            (np.array([100, 50, 50], np.uint8), np.array([130, 255, 255], np.uint8)),
        ],
        "bgr": (255, 0, 0),
    },
}


class VisionSystem:
    def __init__(self, cfg: VisionConfig):
        self.cfg = cfg
        self.cap = cv2.VideoCapture(cfg.camera_index, cfg.camera_backend)
        if not self.cap.isOpened():
            raise RuntimeError("無法開啟攝影機，請確認 camera_index / Iriun Webcam 編號")

        self.world_pts = np.array([
            [0.0, 0.0],
            [cfg.work_width_mm, 0.0],
            [cfg.work_width_mm, cfg.work_height_mm],
            [0.0, cfg.work_height_mm],
        ], dtype=np.float32)

        self._init_aruco()
        self.last_H: Optional[np.ndarray] = None
        self.last_H_inv: Optional[np.ndarray] = None
        self.last_ws_time: float = 0.0
        self.last_image_pts: Optional[np.ndarray] = None

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()

    # ---- ArUco setup (handles old + new API) ----
    def _init_aruco(self):
        self._dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self._params = (cv2.aruco.DetectorParameters()
                        if hasattr(cv2.aruco, "DetectorParameters")
                        else cv2.aruco.DetectorParameters_create())
        self._new_api = hasattr(cv2.aruco, "ArucoDetector")
        self._detector = cv2.aruco.ArucoDetector(self._dict, self._params) if self._new_api else None

    def _detect_markers(self, frame):
        if self._new_api:
            return self._detector.detectMarkers(frame)
        return cv2.aruco.detectMarkers(frame, self._dict, parameters=self._params)

    # ---- workspace (homography) ----
    def _compute_workspace(self, frame) -> bool:
        now = time.monotonic()
        corners, ids, _ = self._detect_markers(frame)

        if ids is None:
            return self._hold_or_clear(now)

        ids_flat = ids.flatten()
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        centers: Dict[int, Tuple[float, float]] = {}
        for mc, mid in zip(corners, ids_flat):
            pts = mc[0]
            cx, cy = float(pts[:, 0].mean()), float(pts[:, 1].mean())
            centers[int(mid)] = (cx, cy)
            cv2.putText(frame, f"ID {int(mid)}", (int(cx)+8, int(cy)-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

        if not all(m in centers for m in self.cfg.required_marker_ids):
            return self._hold_or_clear(now)

        img_pts = np.array([centers[m] for m in self.cfg.required_marker_ids], np.float32)
        self.last_H = cv2.getPerspectiveTransform(img_pts, self.world_pts)
        self.last_H_inv = cv2.getPerspectiveTransform(self.world_pts, img_pts)
        self.last_ws_time = now
        self.last_image_pts = img_pts

        cv2.polylines(frame, [img_pts.astype(np.int32)], True, (0,255,255), 2)
        self._draw_axes(frame)
        return True

    def _hold_or_clear(self, now: float) -> bool:
        if self.last_H is not None and (now - self.last_ws_time) < self.cfg.homography_hold_sec:
            return True
        self.last_H = self.last_H_inv = self.last_image_pts = None
        return False

    def _draw_axes(self, frame):
        if self.last_H_inv is None:
            return
        def w2p(x, y):
            d = cv2.perspectiveTransform(np.float32([[[x,y]]]), self.last_H_inv)
            return int(d[0,0,0]), int(d[0,0,1])
        o  = w2p(0, 0)
        xp = w2p(min(60.0, self.cfg.work_width_mm), 0)
        yp = w2p(0, min(60.0, self.cfg.work_height_mm))
        cv2.circle(frame, o, 8, (255,255,0), -1)
        cv2.putText(frame, "Origin", (o[0]+10, o[1]-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        cv2.arrowedLine(frame, o, xp, (0,255,255), 2, tipLength=0.15)
        cv2.arrowedLine(frame, o, yp, (255,0,255), 2, tipLength=0.15)
        cv2.putText(frame, "+X", (xp[0]+5, xp[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        cv2.putText(frame, "+Y", (yp[0]+5, yp[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,255), 2)

    # ---- pixel <-> world ----
    def _px2world(self, u: float, v: float) -> Tuple[float, float]:
        d = cv2.perspectiveTransform(np.float32([[[u, v]]]), self.last_H)
        return float(d[0,0,0]), float(d[0,0,1])

    # ---- color detection ----
    def _find_target(self, frame) -> Tuple[Optional[dict], Dict[str, np.ndarray]]:
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (11,11), 0), cv2.COLOR_BGR2HSV)
        masks: Dict[str, np.ndarray] = {}
        candidates: List[dict] = []

        for name, info in COLOR_TABLE.items():
            mask = sum(cv2.inRange(hsv, lo, hi) for lo, hi in info["ranges"])
            mask = cv2.dilate(cv2.erode(mask, None, iterations=2), None, iterations=2)
            masks[name] = mask

            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            c = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area < self.cfg.min_contour_area:
                continue
            (cx, cy), r = cv2.minEnclosingCircle(c)
            candidates.append({"name": name, "cx": float(cx), "cy": float(cy),
                               "radius": float(r), "area": float(area)})

        if not candidates:
            return None, masks

        best = max(candidates, key=lambda d: d["area"])
        bgr = COLOR_TABLE[best["name"]]["bgr"]
        bx, by, br = int(best["cx"]), int(best["cy"]), int(best["radius"])
        cv2.circle(frame, (bx, by), br, bgr, 2)
        cv2.circle(frame, (bx, by), 4, (255,255,255), -1)
        cv2.putText(frame, f'{best["name"]} A={int(best["area"])}',
                    (bx-30, by-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr, 2)
        return best, masks

    # ---- main process ----
    def process(self, frame):
        ws = self._compute_workspace(frame)
        best, masks = self._find_target(frame)

        if self.cfg.show_mask:
            for n, m in masks.items():
                cv2.imshow(f"mask_{n.lower()}", m)

        if best is None:
            return None, frame

        wx = wy = None
        if ws and self.last_H is not None:
            wx, wy = self._px2world(best["cx"], best["cy"])
            cv2.putText(frame, f"Pixel=({int(best['cx'])},{int(best['cy'])})",
                        (10,85), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
            cv2.putText(frame, f"World=({wx:.1f},{wy:.1f})mm",
                        (10,115), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,255), 2)

        return TargetObservation(
            color_name=best["name"], pixel_x=int(best["cx"]), pixel_y=int(best["cy"]),
            area=float(best["area"]), world_x=wx, world_y=wy, workspace_ready=ws
        ), frame
