# main.py
from __future__ import annotations

import socket
import cv2

from vision import VisionSystem, VisionConfig # type: ignore
from planner import SortPlanner, PlannerConfig # type: ignore
from kinematics import DemoKinematics, KinematicsConfig, ArmPose # type: ignore


ESP_IP = "192.168.137.232"   # 改成你的 ESP32 IP
ESP_PORT = 8888


class ESP32Client:
    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_text(self, msg: str):
        self.sock.sendto(msg.encode("utf-8"), (self.ip, self.port))

    def send_pose(self, pose: ArmPose):
        msg = f"{pose.base},{pose.shoulder},{pose.elbow},{pose.claw}"
        self.send_text(msg)

    def close(self):
        self.sock.close()


def draw_hud(frame, planner, obs, kin_cfg):
    h = 28
    y = 25

    lines = [
        f"State: {planner.status_text}",
        f"Enabled: {planner.enabled}",
        f"Drop RED={kin_cfg.place_points['RED']}",
        f"Drop GREEN={kin_cfg.place_points['GREEN']}",
        f"Drop BLUE={kin_cfg.place_points['BLUE']}",
        "Keys: 1=start  2=stop  h=home  q=quit"
    ]

    if obs is not None:
        lines.insert(2, f"Target: {obs.color_name}")
        if obs.workspace_ready and obs.world_x is not None and obs.world_y is not None:
            lines.insert(3, f"World: ({obs.world_x:.1f}, {obs.world_y:.1f}) mm")
        else:
            lines.insert(3, "World: workspace not ready")
    else:
        lines.insert(2, "Target: none")
        lines.insert(3, "World: none")

    for line in lines:
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
        y += h


def main():
    vision_cfg = VisionConfig(
        camera_index=0,            # Iriun 若不是 0 改 1 或 2
        work_width_mm=300.0,       # 改成你的工作區寬
        work_height_mm=200.0,      # 改成你的工作區高
        required_marker_ids=(0, 1, 2, 3),
        min_contour_area=400.0,
        show_mask=False
    )

    kin_cfg = KinematicsConfig(
        work_width_mm=300.0,
        work_height_mm=200.0,
        place_points={
            "RED": (260.0, 160.0),
            "GREEN": (260.0, 100.0),
            "BLUE": (260.0, 40.0),
        }
    )

    planner_cfg = PlannerConfig(
        stable_frames_required=8,
        stable_distance_mm=12.0,
        dwell_pick_above_sec=0.9,
        dwell_pick_down_sec=0.9,
        dwell_grab_sec=0.8,
        dwell_lift_sec=0.9,
        dwell_place_above_sec=0.9,
        dwell_place_down_sec=0.9,
        dwell_release_sec=0.8,
        dwell_return_sec=1.0
    )

    vision = VisionSystem(vision_cfg)
    kin = DemoKinematics(kin_cfg)
    planner = SortPlanner(planner_cfg)
    esp = ESP32Client(ESP_IP, ESP_PORT)

    print("操作說明：")
    print("1 = 啟動自動分揀")
    print("2 = 停止")
    print("h = 回 Home")
    print("q = 離開")
    print("ArUco 角點 ID 對應：0=左下, 1=右下, 2=右上, 3=左上")

    last_sent_pose = None

    try:
        esp.send_text("STOP")

        while True:
            ret, frame = vision.read()
            if not ret:
                print("讀不到攝影機畫面，程式結束")
                break

            obs, frame = vision.process(frame)

            next_pose = planner.update(obs, kin)
            if next_pose is not None:
                esp.send_pose(next_pose)
                last_sent_pose = next_pose

            draw_hud(frame, planner, obs, kin_cfg)

            if last_sent_pose is not None:
                cv2.putText(
                    frame,
                    f"LastPose: {last_sent_pose.name} -> {last_sent_pose.as_tuple()}",
                    (10, frame.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2
                )

            cv2.imshow("Color Sorting Demo", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('1'):
                planner.start()
                esp.send_text("START")
                home_pose = kin.home()
                esp.send_pose(home_pose)
                last_sent_pose = home_pose

            elif key == ord('2'):
                planner.stop()
                esp.send_text("STOP")

            elif key == ord('h'):
                home_pose = kin.home()
                esp.send_text("START")
                esp.send_pose(home_pose)
                last_sent_pose = home_pose

            elif key == ord('q'):
                break

    finally:
        esp.send_text("STOP")
        vision.release()
        esp.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
