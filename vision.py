# vision_commented.py
# 這個檔案負責所有「看」的工作：開相機、抓 ArUco、抓顏色、算世界座標。

from __future__ import annotations  # 讓型別註記可以參考尚未宣告完成的型別名稱。

from dataclasses import dataclass  # 用 dataclass 讓資料結構更乾淨好讀。
from typing import Optional, Dict, Tuple, List  # 匯入型別工具，方便做清楚的型別標註。
import time  # 用來記錄時間，控制 homography 暫時保留等功能。
import cv2  # OpenCV 主函式庫，負責影像處理與 ArUco 偵測。
import numpy as np  # NumPy 負責矩陣與座標資料處理。


@dataclass  # 這個裝飾器讓這個 class 變成簡潔的設定資料容器。
class VisionConfig:
    camera_index: int = 0  # 預設使用編號 0 的攝影機。
    camera_backend: int = cv2.CAP_DSHOW if hasattr(cv2, "CAP_DSHOW") else 0  # Windows 常用 DSHOW，其他平台沒有就退回 0。
    work_width_mm: float = 300.0  # 工作區實際寬度，單位 mm。
    work_height_mm: float = 200.0  # 工作區實際高度，單位 mm。
    required_marker_ids: Tuple[int, int, int, int] = (0, 1, 2, 3)  # 四個 ArUco 的 ID 順序：左下、右下、右上、左上。
    min_contour_area: float = 400.0  # 顏色輪廓最小面積，小於這個值的雜訊會被忽略。
    homography_hold_sec: float = 0.8  # ArUco 短暫消失時，暫時保留上一組 homography 的秒數。
    show_mask: bool = False  # 是否另外顯示紅綠藍的二值遮罩視窗。


@dataclass  # 這個 dataclass 用來打包一次偵測結果。
class TargetObservation:
    color_name: str  # 偵測到的顏色名稱，例如 RED。
    pixel_x: int  # 目標中心點在畫面中的 x 像素。
    pixel_y: int  # 目標中心點在畫面中的 y 像素。
    area: float  # 目標輪廓面積。
    world_x: Optional[float]  # 目標在工作區的 x 座標，若未完成標定則可能是 None。
    world_y: Optional[float]  # 目標在工作區的 y 座標，若未完成標定則可能是 None。
    workspace_ready: bool  # 工作區是否已經成功建立 homography。


class VisionSystem:
    # 建構子：初始化攝影機、顏色範圍、ArUco 參數與狀態變數。
    def __init__(self, config: VisionConfig):
        self.cfg = config  # 把外部傳入的設定存起來。
        self.cap = cv2.VideoCapture(self.cfg.camera_index, self.cfg.camera_backend)  # 開啟攝影機。

        if not self.cap.isOpened():  # 如果攝影機打不開。
            raise RuntimeError("無法開啟攝影機，請確認 camera_index / Iriun Webcam 編號")  # 直接報錯提醒使用者。

        self.world_pts = np.array([  # 定義工作區四個角在真實世界中的座標。
            [0.0, 0.0],  # ID 0 -> 左下角。
            [self.cfg.work_width_mm, 0.0],  # ID 1 -> 右下角。
            [self.cfg.work_width_mm, self.cfg.work_height_mm],  # ID 2 -> 右上角。
            [0.0, self.cfg.work_height_mm],  # ID 3 -> 左上角。
        ], dtype=np.float32)  # 用 float32 是 OpenCV 幾何轉換常用格式。

        self.lower_red1 = np.array([0, 120, 70], dtype=np.uint8)  # 紅色第一段 HSV 下界。
        self.upper_red1 = np.array([10, 255, 255], dtype=np.uint8)  # 紅色第一段 HSV 上界。
        self.lower_red2 = np.array([170, 120, 70], dtype=np.uint8)  # 紅色第二段 HSV 下界，因為紅色跨 HSV 頭尾。
        self.upper_red2 = np.array([180, 255, 255], dtype=np.uint8)  # 紅色第二段 HSV 上界。

        self.lower_green = np.array([40, 50, 50], dtype=np.uint8)  # 綠色 HSV 下界。
        self.upper_green = np.array([80, 255, 255], dtype=np.uint8)  # 綠色 HSV 上界。

        self.lower_blue = np.array([100, 50, 50], dtype=np.uint8)  # 藍色 HSV 下界。
        self.upper_blue = np.array([130, 255, 255], dtype=np.uint8)  # 藍色 HSV 上界。

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  # 選用 DICT_4X4_50 字典。
        if hasattr(cv2.aruco, "DetectorParameters"):  # 新版 OpenCV 會有這個 class。
            self.aruco_params = cv2.aruco.DetectorParameters()  # 建立新版參數物件。
        else:  # 舊版 OpenCV 走這個分支。
            self.aruco_params = cv2.aruco.DetectorParameters_create()  # 建立舊版參數物件。

        self.use_new_aruco_api = hasattr(cv2.aruco, "ArucoDetector")  # 檢查是不是新版 ArUco API。
        if self.use_new_aruco_api:  # 如果是新版 API。
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)  # 建立新版 detector。
        else:  # 如果是舊版 API。
            self.aruco_detector = None  # 舊版不需要額外 detector 物件。

        self.last_H: Optional[np.ndarray] = None  # 上一次成功建立的 homography。
        self.last_H_inv: Optional[np.ndarray] = None  # 上一次 homography 的反矩陣。
        self.last_workspace_seen_time: float = 0.0  # 上一次成功看見完整工作區的時間。
        self.last_image_pts: Optional[np.ndarray] = None  # 上一次偵測到的工作區四角像素座標。

    def read(self):  # 封裝讀取一張畫面。
        return self.cap.read()  # 直接呼叫 OpenCV 的 read。

    def release(self):  # 封裝釋放攝影機。
        self.cap.release()  # 釋放攝影機資源。

    def _detect_markers(self, frame):  # 內部函式：負責偵測 ArUco marker。
        if self.use_new_aruco_api:  # 新版 OpenCV。
            corners, ids, rejected = self.aruco_detector.detectMarkers(frame)  # 用新版 detector 偵測 marker。
        else:  # 舊版 OpenCV。
            corners, ids, rejected = cv2.aruco.detectMarkers(  # 直接呼叫舊版函式。
                frame, self.aruco_dict, parameters=self.aruco_params
            )
        return corners, ids, rejected  # 回傳角點、ID、被拒絕的候選。

    def _compute_workspace(self, frame) -> bool:  # 內部函式：根據 ArUco 算出工作區 homography。
        now = time.monotonic()  # 取得目前單調時間，用來比較時間差。
        corners, ids, _ = self._detect_markers(frame)  # 偵測目前畫面中的 ArUco marker。

        if ids is None:  # 如果一個 marker 都沒找到。
            if self.last_H is not None and (now - self.last_workspace_seen_time) < self.cfg.homography_hold_sec:  # 如果剛剛才看過完整工作區。
                return True  # 仍暫時沿用上一組 homography，避免畫面短暫抖動就失效。
            self.last_H = None  # 否則清掉 homography。
            self.last_H_inv = None  # 清掉反矩陣。
            self.last_image_pts = None  # 清掉上一組影像點。
            return False  # 回報工作區尚未就緒。

        ids = ids.flatten()  # 把 ids 攤平成一維陣列，方便後面操作。
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)  # 在畫面上畫出 marker 外框與 ID。

        marker_centers: Dict[int, Tuple[float, float]] = {}  # 建立字典來存每個 marker 的中心點。
        for marker_corners, marker_id in zip(corners, ids):  # 一個一個處理偵測到的 marker。
            pts = marker_corners[0]  # 取出這個 marker 的四個角點。
            cx = float(np.mean(pts[:, 0]))  # 取 x 平均當中心點。
            cy = float(np.mean(pts[:, 1]))  # 取 y 平均當中心點。
            marker_centers[int(marker_id)] = (cx, cy)  # 存進字典，key 是 marker ID。

            cv2.putText(  # 在畫面上把 marker ID 顯示出來。
                frame, f"ID {int(marker_id)}",
                (int(cx) + 8, int(cy) - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2
            )

        if not all(mid in marker_centers for mid in self.cfg.required_marker_ids):  # 如果四個必要 ID 沒有全部出現。
            if self.last_H is not None and (now - self.last_workspace_seen_time) < self.cfg.homography_hold_sec:  # 但剛剛才有看見完整工作區。
                return True  # 就暫時保留上一組 homography。
            self.last_H = None  # 否則清除 homography。
            self.last_H_inv = None  # 清除反矩陣。
            self.last_image_pts = None  # 清除影像點。
            return False  # 回報工作區尚未就緒。

        image_pts = np.array([  # 按照固定順序組出影像四角點。
            marker_centers[self.cfg.required_marker_ids[0]],  # 左下。
            marker_centers[self.cfg.required_marker_ids[1]],  # 右下。
            marker_centers[self.cfg.required_marker_ids[2]],  # 右上。
            marker_centers[self.cfg.required_marker_ids[3]],  # 左上。
        ], dtype=np.float32)  # 轉成 float32 給 OpenCV 使用。

        self.last_H = cv2.getPerspectiveTransform(image_pts, self.world_pts)  # 算影像 -> 世界的透視轉換矩陣。
        self.last_H_inv = cv2.getPerspectiveTransform(self.world_pts, image_pts)  # 算世界 -> 影像的反向轉換矩陣。
        self.last_workspace_seen_time = now  # 記錄這次成功標定的時間。
        self.last_image_pts = image_pts  # 把影像點存起來，方便後續檢查或除錯。

        pts_int = image_pts.astype(np.int32)  # 轉成整數，因為畫圖需要整數座標。
        cv2.polylines(frame, [pts_int], isClosed=True, color=(0, 255, 255), thickness=2)  # 在畫面畫出工作區輪廓。
        self._draw_origin_axes(frame)  # 額外把原點和座標軸方向畫出來。

        return True  # 回報工作區已就緒。

    def _draw_origin_axes(self, frame):  # 內部函式：把工作區原點與 X/Y 軸方向畫在畫面上。
        if self.last_H_inv is None:  # 若沒有反矩陣，代表還沒完成工作區標定。
            return  # 直接不畫。

        def world_to_pixel(x, y):  # 小工具函式：世界座標 -> 畫面像素。
            src = np.array([[[x, y]]], dtype=np.float32)  # 包成 OpenCV 需要的 shape。
            dst = cv2.perspectiveTransform(src, self.last_H_inv)  # 做透視轉換。
            return int(dst[0][0][0]), int(dst[0][0][1])  # 回傳整數像素點。

        origin = world_to_pixel(0, 0)  # 世界座標原點。
        x_axis = world_to_pixel(min(60.0, self.cfg.work_width_mm), 0)  # X 軸方向上的一個參考點。
        y_axis = world_to_pixel(0, min(60.0, self.cfg.work_height_mm))  # Y 軸方向上的一個參考點。

        cv2.circle(frame, origin, 8, (255, 255, 0), -1)  # 畫出原點。
        cv2.putText(frame, "Origin (0,0)", (origin[0] + 10, origin[1] - 8),  # 在原點旁邊標字。
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.arrowedLine(frame, origin, x_axis, (0, 255, 255), 2, tipLength=0.15)  # 畫 X 軸箭頭。
        cv2.arrowedLine(frame, origin, y_axis, (255, 0, 255), 2, tipLength=0.15)  # 畫 Y 軸箭頭。
        cv2.putText(frame, "+X", (x_axis[0] + 5, x_axis[1]),  # 標記 X 軸。
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, "+Y", (y_axis[0] + 5, y_axis[1]),  # 標記 Y 軸。
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

    def _pixel_to_world(self, u: float, v: float) -> Tuple[float, float]:  # 內部函式：畫面像素 -> 世界座標。
        if self.last_H is None:  # 如果還沒有 homography。
            raise RuntimeError("Homography 尚未建立")  # 直接報錯提醒。
        src = np.array([[[u, v]]], dtype=np.float32)  # 包成 OpenCV 需要的格式。
        dst = cv2.perspectiveTransform(src, self.last_H)  # 做透視轉換。
        return float(dst[0][0][0]), float(dst[0][0][1])  # 回傳 x, y 世界座標。

    def _find_best_color_target(self, frame) -> Tuple[Optional[dict], Dict[str, np.ndarray]]:  # 內部函式：找出畫面中最主要的紅/綠/藍目標。
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)  # 先模糊化，減少雜訊。
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)  # 把 BGR 轉成 HSV，顏色分割更穩定。

        mask_r = cv2.inRange(hsv, self.lower_red1, self.upper_red1) + \
                 cv2.inRange(hsv, self.lower_red2, self.upper_red2)  # 紅色用兩段 mask 相加。
        mask_g = cv2.inRange(hsv, self.lower_green, self.upper_green)  # 綠色 mask。
        mask_b = cv2.inRange(hsv, self.lower_blue, self.upper_blue)  # 藍色 mask。

        masks = {  # 把三種顏色的 mask 收起來，方便後面統一處理。
            "RED": mask_r,
            "GREEN": mask_g,
            "BLUE": mask_b
        }

        candidates: List[dict] = []  # 這裡用來存所有可能的顏色候選目標。

        for color_name, raw_mask in masks.items():  # 逐一處理紅、綠、藍。
            mask = cv2.erode(raw_mask, None, iterations=2)  # 先侵蝕，去除小雜點。
            mask = cv2.dilate(mask, None, iterations=2)  # 再膨脹，把主體補回來。
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 找輪廓。

            if len(contours) == 0:  # 如果這個顏色沒找到任何輪廓。
                continue  # 換下一個顏色。

            c = max(contours, key=cv2.contourArea)  # 取這個顏色中最大的輪廓當主要目標。
            area = cv2.contourArea(c)  # 算輪廓面積。
            if area < self.cfg.min_contour_area:  # 太小代表可能是雜訊。
                continue  # 忽略這個候選。

            (cx, cy), radius = cv2.minEnclosingCircle(c)  # 算最小外接圓的圓心和半徑。
            candidates.append({  # 把候選資訊存起來。
                "name": color_name,
                "cx": float(cx),
                "cy": float(cy),
                "radius": float(radius),
                "area": float(area),
                "mask": mask
            })

        if not candidates:  # 如果三種顏色都沒有合格候選。
            return None, masks  # 回傳沒有目標，但仍回傳 masks 供需要時顯示。

        best = max(candidates, key=lambda x: x["area"])  # 從所有候選中選面積最大的當目標。
        draw_color = {  # 為了畫圖，先定義不同顏色的 BGR 繪圖色。
            "RED": (0, 0, 255),
            "GREEN": (0, 255, 0),
            "BLUE": (255, 0, 0)
        }[best["name"]]

        cv2.circle(frame, (int(best["cx"]), int(best["cy"])), int(best["radius"]), draw_color, 2)  # 在目標外圍畫圓。
        cv2.circle(frame, (int(best["cx"]), int(best["cy"])), 4, (255, 255, 255), -1)  # 在中心點畫白點。
        cv2.putText(frame, f'{best["name"]} A={int(best["area"])}',  # 顯示顏色名稱與面積。
                    (int(best["cx"]) - 30, int(best["cy"]) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)

        return best, masks  # 回傳最佳候選與所有 mask。

    def process(self, frame):  # 對一張畫面做完整處理：工作區定位 + 顏色偵測。
        workspace_ready = self._compute_workspace(frame)  # 先更新工作區 homography 狀態。
        best_target, masks = self._find_best_color_target(frame)  # 再找顏色目標。

        if self.cfg.show_mask:  # 如果設定要顯示 mask。
            for name, mask in masks.items():  # 把紅綠藍 mask 一個一個顯示出來。
                cv2.imshow(f"mask_{name.lower()}", mask)

        if best_target is None:  # 如果沒找到任何顏色目標。
            return None, frame  # 回傳 None 和原始畫面（已加上一些可能的標記）。

        world_x = None  # 預設世界座標 x 為 None。
        world_y = None  # 預設世界座標 y 為 None。

        if workspace_ready and self.last_H is not None:  # 只有工作區已就緒時，才能把像素轉成世界座標。
            world_x, world_y = self._pixel_to_world(best_target["cx"], best_target["cy"])  # 轉換目標中心點。
            cv2.putText(frame, f"Pixel=({int(best_target['cx'])}, {int(best_target['cy'])})",  # 在畫面顯示像素座標。
                        (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            cv2.putText(frame, f"World=({world_x:.1f}, {world_y:.1f}) mm",  # 在畫面顯示世界座標。
                        (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

        obs = TargetObservation(  # 把本次結果包成 TargetObservation。
            color_name=best_target["name"],
            pixel_x=int(best_target["cx"]),
            pixel_y=int(best_target["cy"]),
            area=float(best_target["area"]),
            world_x=world_x,
            world_y=world_y,
            workspace_ready=workspace_ready
        )
        return obs, frame  # 回傳觀測結果和畫上資訊的畫面。
