# main.py
from __future__ import annotations
import socket, cv2 # type: ignore

from vision import VisionSystem, VisionConfig        # type: ignore
from planner import SortPlanner, PlannerConfig        # type: ignore
from kinematics import DemoKinematics, KinematicsConfig, ArmPose  # type: ignore

ESP_IP   = "192.168.137.232"
ESP_PORT = 8888


class ESP32Client:
    def __init__(self, ip: str, port: int):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, msg: str):
        self.sock.sendto(msg.encode(), self.addr)

    def send_pose(self, p: ArmPose):
        self.send(f"{p.base},{p.shoulder},{p.elbow},{p.claw}")

    def close(self):
        self.sock.close()


KEY_ACTIONS = {
    ord("1"): "start",
    ord("2"): "stop",
    ord("h"): "home",
    ord("q"): "quit",
}


def draw_hud(frame, planner, obs, kin_cfg):
    lines = [
        f"State: {planner.status_text}",
        f"Enabled: {planner.enabled}",
    ]
    if obs and obs.workspace_ready and obs.world_x is not None:
        lines.append(f"Target: {obs.color_name}  World: ({obs.world_x:.1f}, {obs.world_y:.1f}) mm")
    elif obs:
        lines.append(f"Target: {obs.color_name}  World: not ready")
    else:
        lines.append("Target: none")
    for color, pt in kin_cfg.place_points.items():
        lines.append(f"Drop {color}={pt}")
    lines.append("Keys: 1=start 2=stop h=home q=quit")

    y = 25
    for ln in lines:
        cv2.putText(frame, ln, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255,255,255), 2)
        y += 28


def main():
    vision_cfg  = VisionConfig(camera_index=0, work_width_mm=300.0, work_height_mm=200.0,
                               min_contour_area=400.0, show_mask=False)
    kin_cfg     = KinematicsConfig(work_width_mm=300.0, work_height_mm=200.0)
    planner_cfg = PlannerConfig()

    vision  = VisionSystem(vision_cfg)
    kin     = DemoKinematics(kin_cfg)
    planner = SortPlanner(planner_cfg)
    esp     = ESP32Client(ESP_IP, ESP_PORT)
    last_pose: ArmPose | None = None

    print("操作說明：1=啟動  2=停止  h=Home  q=離開")
    esp.send("STOP")

    try:
        while True:
            ret, frame = vision.read()
            if not ret:
                print("讀不到攝影機畫面，程式結束")
                break

            obs, frame = vision.process(frame)
            pose = planner.update(obs, kin)
            if pose:
                esp.send_pose(pose)
                last_pose = pose

            draw_hud(frame, planner, obs, kin_cfg)
            if last_pose:
                cv2.putText(frame, f"LastPose: {last_pose.name} -> {last_pose.as_tuple()}",
                            (10, frame.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0,255,255), 2)

            cv2.imshow("Color Sorting Demo", frame)
            action = KEY_ACTIONS.get(cv2.waitKey(1) & 0xFF)

            if action == "start":
                planner.start()
                esp.send("START")
                home = kin.home()
                esp.send_pose(home)
                last_pose = home
            elif action == "stop":
                planner.stop()
                esp.send("STOP")
            elif action == "home":
                home = kin.home()
                esp.send("START")
                esp.send_pose(home)
                last_pose = home
            elif action == "quit":
                break

    finally:
        esp.send("STOP")
        vision.release()
        esp.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
