# vision.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Dict, Tuple, List
import time
import cv2
import numpy as np


@dataclass
class VisionConfig:
    camera_index: int = 0
    camera_backend: int = cv2.CAP_DSHOW if hasattr(cv2, "CAP_DSHOW") else 0
    work_width_mm: float = 300.0
    work_height_mm: float = 200.0
    required_marker_ids: Tuple[int, int, int, int] = (0, 1, 2, 3)  # 左下, 右下, 右上, 左上
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


class VisionSystem:
    def __init__(self, config: VisionConfig):
        self.cfg = config
        self.cap = cv2.VideoCapture(self.cfg.camera_index, self.cfg.camera_backend)

        if not self.cap.isOpened():
            raise RuntimeError("無法開啟攝影機，請確認 camera_index / Iriun Webcam 編號")

        self.world_pts = np.array([
            [0.0, 0.0],                                # ID 0 -> 左下
            [self.cfg.work_width_mm, 0.0],            # ID 1 -> 右下
            [self.cfg.work_width_mm, self.cfg.work_height_mm],  # ID 2 -> 右上
            [0.0, self.cfg.work_height_mm],           # ID 3 -> 左上
        ], dtype=np.float32)

        self.lower_red1 = np.array([0, 120, 70], dtype=np.uint8)
        self.upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
        self.lower_red2 = np.array([170, 120, 70], dtype=np.uint8)
        self.upper_red2 = np.array([180, 255, 255], dtype=np.uint8)

        self.lower_green = np.array([40, 50, 50], dtype=np.uint8)
        self.upper_green = np.array([80, 255, 255], dtype=np.uint8)

        self.lower_blue = np.array([100, 50, 50], dtype=np.uint8)
        self.upper_blue = np.array([130, 255, 255], dtype=np.uint8)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.use_new_aruco_api = hasattr(cv2.aruco, "ArucoDetector")
        if self.use_new_aruco_api:
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.aruco_detector = None

        self.last_H: Optional[np.ndarray] = None
        self.last_H_inv: Optional[np.ndarray] = None
        self.last_workspace_seen_time: float = 0.0
        self.last_image_pts: Optional[np.ndarray] = None

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()

    def _detect_markers(self, frame):
        if self.use_new_aruco_api:
            corners, ids, rejected = self.aruco_detector.detectMarkers(frame)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                frame, self.aruco_dict, parameters=self.aruco_params
            )
        return corners, ids, rejected

    def _compute_workspace(self, frame) -> bool:
        now = time.monotonic()
        corners, ids, _ = self._detect_markers(frame)

        if ids is None:
            if self.last_H is not None and (now - self.last_workspace_seen_time) < self.cfg.homography_hold_sec:
                return True
            self.last_H = None
            self.last_H_inv = None
            self.last_image_pts = None
            return False

        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        marker_centers: Dict[int, Tuple[float, float]] = {}
        for marker_corners, marker_id in zip(corners, ids):
            pts = marker_corners[0]
            cx = float(np.mean(pts[:, 0]))
            cy = float(np.mean(pts[:, 1]))
            marker_centers[int(marker_id)] = (cx, cy)

            cv2.putText(
                frame, f"ID {int(marker_id)}",
                (int(cx) + 8, int(cy) - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2
            )

        if not all(mid in marker_centers for mid in self.cfg.required_marker_ids):
            if self.last_H is not None and (now - self.last_workspace_seen_time) < self.cfg.homography_hold_sec:
                return True
            self.last_H = None
            self.last_H_inv = None
            self.last_image_pts = None
            return False

        image_pts = np.array([
            marker_centers[self.cfg.required_marker_ids[0]],
            marker_centers[self.cfg.required_marker_ids[1]],
            marker_centers[self.cfg.required_marker_ids[2]],
            marker_centers[self.cfg.required_marker_ids[3]],
        ], dtype=np.float32)

        self.last_H = cv2.getPerspectiveTransform(image_pts, self.world_pts)
        self.last_H_inv = cv2.getPerspectiveTransform(self.world_pts, image_pts)
        self.last_workspace_seen_time = now
        self.last_image_pts = image_pts

        pts_int = image_pts.astype(np.int32)
        cv2.polylines(frame, [pts_int], isClosed=True, color=(0, 255, 255), thickness=2)
        self._draw_origin_axes(frame)

        return True

    def _draw_origin_axes(self, frame):
        if self.last_H_inv is None:
            return

        def world_to_pixel(x, y):
            src = np.array([[[x, y]]], dtype=np.float32)
            dst = cv2.perspectiveTransform(src, self.last_H_inv)
            return int(dst[0][0][0]), int(dst[0][0][1])

        origin = world_to_pixel(0, 0)
        x_axis = world_to_pixel(min(60.0, self.cfg.work_width_mm), 0)
        y_axis = world_to_pixel(0, min(60.0, self.cfg.work_height_mm))

        cv2.circle(frame, origin, 8, (255, 255, 0), -1)
        cv2.putText(frame, "Origin (0,0)", (origin[0] + 10, origin[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.arrowedLine(frame, origin, x_axis, (0, 255, 255), 2, tipLength=0.15)
        cv2.arrowedLine(frame, origin, y_axis, (255, 0, 255), 2, tipLength=0.15)
        cv2.putText(frame, "+X", (x_axis[0] + 5, x_axis[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, "+Y", (y_axis[0] + 5, y_axis[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

    def _pixel_to_world(self, u: float, v: float) -> Tuple[float, float]:
        if self.last_H is None:
            raise RuntimeError("Homography 尚未建立")
        src = np.array([[[u, v]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(src, self.last_H)
        return float(dst[0][0][0]), float(dst[0][0][1])

    def _find_best_color_target(self, frame) -> Tuple[Optional[dict], Dict[str, np.ndarray]]:
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask_r = cv2.inRange(hsv, self.lower_red1, self.upper_red1) + \
                 cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_g = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask_b = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

        masks = {
            "RED": mask_r,
            "GREEN": mask_g,
            "BLUE": mask_b
        }

        candidates: List[dict] = []

        for color_name, raw_mask in masks.items():
            mask = cv2.erode(raw_mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) == 0:
                continue

            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area < self.cfg.min_contour_area:
                continue

            (cx, cy), radius = cv2.minEnclosingCircle(c)
            candidates.append({
                "name": color_name,
                "cx": float(cx),
                "cy": float(cy),
                "radius": float(radius),
                "area": float(area),
                "mask": mask
            })

        if not candidates:
            return None, masks

        best = max(candidates, key=lambda x: x["area"])
        draw_color = {
            "RED": (0, 0, 255),
            "GREEN": (0, 255, 0),
            "BLUE": (255, 0, 0)
        }[best["name"]]

        cv2.circle(frame, (int(best["cx"]), int(best["cy"])), int(best["radius"]), draw_color, 2)
        cv2.circle(frame, (int(best["cx"]), int(best["cy"])), 4, (255, 255, 255), -1)
        cv2.putText(frame, f'{best["name"]} A={int(best["area"])}',
                    (int(best["cx"]) - 30, int(best["cy"]) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)

        return best, masks

    def process(self, frame):
        workspace_ready = self._compute_workspace(frame)
        best_target, masks = self._find_best_color_target(frame)

        if self.cfg.show_mask:
            for name, mask in masks.items():
                cv2.imshow(f"mask_{name.lower()}", mask)

        if best_target is None:
            return None, frame

        world_x = None
        world_y = None

        if workspace_ready and self.last_H is not None:
            world_x, world_y = self._pixel_to_world(best_target["cx"], best_target["cy"])
            cv2.putText(frame, f"Pixel=({int(best_target['cx'])}, {int(best_target['cy'])})",
                        (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            cv2.putText(frame, f"World=({world_x:.1f}, {world_y:.1f}) mm",
                        (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

        obs = TargetObservation(
            color_name=best_target["name"],
            pixel_x=int(best_target["cx"]),
            pixel_y=int(best_target["cy"]),
            area=float(best_target["area"]),
            world_x=world_x,
            world_y=world_y,
            workspace_ready=workspace_ready
        )
        return obs, frame
