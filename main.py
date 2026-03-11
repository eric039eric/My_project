# main_commented.py
# 這個檔案是總控程式：把 vision、planner、kinematics 串起來，並用 UDP 把角度送給 ESP32。

from __future__ import annotations  # 允許前向型別參考。

import socket  # 用 UDP socket 和 ESP32 通訊。
import cv2  # 用來顯示影像和處理鍵盤事件。

from vision_commented import VisionSystem, VisionConfig  # 匯入視覺系統與設定。
from planner_commented import SortPlanner, PlannerConfig  # 匯入流程規劃器與設定。
from kinematics_commented import DemoKinematics, KinematicsConfig, ArmPose  # 匯入運動學與姿態型別。


ESP_IP = "192.168.137.232"   # 改成你自己的 ESP32 IP 位址。
ESP_PORT = 8888  # UDP port，需和 ESP32 端一致。


class ESP32Client:
    # 這個 class 專門負責把文字或姿態送給 ESP32。
    def __init__(self, ip: str, port: int):
        self.ip = ip  # 存 IP。
        self.port = port  # 存 port。
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 建立 UDP socket。

    def send_text(self, msg: str):  # 傳送純文字命令，例如 START / STOP / HOME。
        self.sock.sendto(msg.encode("utf-8"), (self.ip, self.port))  # 把字串編碼後送出。

    def send_pose(self, pose: ArmPose):  # 傳送四軸姿態。
        msg = f"{pose.base},{pose.shoulder},{pose.elbow},{pose.claw}"  # 組成 ESP32 端能解析的格式。
        self.send_text(msg)  # 直接沿用 send_text 傳出去。

    def close(self):  # 關閉 socket。
        self.sock.close()  # 釋放網路資源。


def draw_hud(frame, planner, obs, kin_cfg):  # 把狀態資訊畫在畫面左上角。
    h = 28  # 每一行文字的垂直間距。
    y = 25  # 第一行文字起始 y 座標。

    lines = [  # 先建立固定顯示的文字列表。
        f"State: {planner.status_text}",  # 當前狀態。
        f"Enabled: {planner.enabled}",  # 是否已啟動自動流程。
        f"Drop RED={kin_cfg.place_points['RED']}",  # 紅色放置點。
        f"Drop GREEN={kin_cfg.place_points['GREEN']}",  # 綠色放置點。
        f"Drop BLUE={kin_cfg.place_points['BLUE']}",  # 藍色放置點。
        "Keys: 1=start  2=stop  h=home  q=quit"  # 按鍵提示。
    ]

    if obs is not None:  # 如果目前有偵測到目標。
        lines.insert(2, f"Target: {obs.color_name}")  # 插入顏色資訊。
        if obs.workspace_ready and obs.world_x is not None and obs.world_y is not None:  # 如果工作區也已就緒。
            lines.insert(3, f"World: ({obs.world_x:.1f}, {obs.world_y:.1f}) mm")  # 顯示世界座標。
        else:  # 如果工作區還沒準備好。
            lines.insert(3, "World: workspace not ready")  # 提示工作區未就緒。
    else:  # 如果目前沒有看到目標。
        lines.insert(2, "Target: none")  # 顯示沒有目標。
        lines.insert(3, "World: none")  # 世界座標也沒有。

    for line in lines:  # 把每一行文字畫上去。
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)  # 畫文字。
        y += h  # 下一行往下移。


def main():  # 主程式入口。
    vision_cfg = VisionConfig(  # 建立視覺系統設定。
        camera_index=0,  # 預設使用 0 號攝影機；如果是 Iriun 可能要改 1 或 2。
        work_width_mm=300.0,  # 工作區寬度。
        work_height_mm=200.0,  # 工作區高度。
        required_marker_ids=(0, 1, 2, 3),  # 四角 ArUco ID 固定順序。
        min_contour_area=400.0,  # 顏色輪廓面積閾值。
        show_mask=False  # 預設不顯示 mask 視窗。
    )

    kin_cfg = KinematicsConfig(  # 建立運動學設定。
        work_width_mm=300.0,  # 工作區寬度。
        work_height_mm=200.0,  # 工作區高度。
        place_points={  # 三個顏色對應的放置點座標。
            "RED": (260.0, 160.0),
            "GREEN": (260.0, 100.0),
            "BLUE": (260.0, 40.0),
        }
    )

    planner_cfg = PlannerConfig(  # 建立狀態機設定。
        stable_frames_required=8,  # 至少穩定出現 8 幀才鎖定。
        stable_distance_mm=12.0,  # 允許的位置浮動範圍。
        dwell_pick_above_sec=0.9,  # 各狀態停留時間。
        dwell_pick_down_sec=0.9,
        dwell_grab_sec=0.8,
        dwell_lift_sec=0.9,
        dwell_place_above_sec=0.9,
        dwell_place_down_sec=0.9,
        dwell_release_sec=0.8,
        dwell_return_sec=1.0
    )

    vision = VisionSystem(vision_cfg)  # 建立視覺系統。
    kin = DemoKinematics(kin_cfg)  # 建立運動學系統。
    planner = SortPlanner(planner_cfg)  # 建立狀態機規劃器。
    esp = ESP32Client(ESP_IP, ESP_PORT)  # 建立 ESP32 UDP 客戶端。

    print("操作說明：")  # 在終端機提示操作方式。
    print("1 = 啟動自動分揀")
    print("2 = 停止")
    print("h = 回 Home")
    print("q = 離開")
    print("ArUco 角點 ID 對應：0=左下, 1=右下, 2=右上, 3=左上")

    last_sent_pose = None  # 記錄最後一次送出的姿態，方便顯示在畫面上。

    try:  # 用 try/finally 確保程式中斷時仍能做清理。
        esp.send_text("STOP")  # 一開始先送 STOP，確保機械手臂不要亂動。

        while True:  # 主迴圈：一幀一幀地持續處理。
            ret, frame = vision.read()  # 從攝影機讀一張畫面。
            if not ret:  # 如果讀取失敗。
                print("讀不到攝影機畫面，程式結束")  # 印出錯誤訊息。
                break  # 跳出主迴圈。

            obs, frame = vision.process(frame)  # 讓 vision 模組處理這一幀，取得觀測結果與標註後畫面。

            next_pose = planner.update(obs, kin)  # 把觀測結果交給 planner，決定是否要切換到下一個姿態。
            if next_pose is not None:  # 如果 planner 回傳了一個新姿態。
                esp.send_pose(next_pose)  # 把姿態送到 ESP32。
                last_sent_pose = next_pose  # 更新最後送出的姿態。

            draw_hud(frame, planner, obs, kin_cfg)  # 在畫面上畫出狀態資訊。

            if last_sent_pose is not None:  # 如果曾經送過姿態。
                cv2.putText(  # 在畫面下方顯示最後送出的姿態。
                    frame,
                    f"LastPose: {last_sent_pose.name} -> {last_sent_pose.as_tuple()}",
                    (10, frame.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2
                )

            cv2.imshow("Color Sorting Demo", frame)  # 顯示畫面視窗。
            key = cv2.waitKey(1) & 0xFF  # 讀取鍵盤輸入。

            if key == ord('1'):  # 如果按下 1。
                planner.start()  # 啟動自動流程。
                esp.send_text("START")  # 通知 ESP32 系統開始。
                home_pose = kin.home()  # 先取得 Home 姿態。
                esp.send_pose(home_pose)  # 把手臂先送回 Home。
                last_sent_pose = home_pose  # 更新最後姿態。

            elif key == ord('2'):  # 如果按下 2。
                planner.stop()  # 停止狀態機。
                esp.send_text("STOP")  # 通知 ESP32 停止。

            elif key == ord('h'):  # 如果按下 h。
                home_pose = kin.home()  # 取得 Home 姿態。
                esp.send_text("START")  # 先讓 ESP32 進入可動作狀態。
                esp.send_pose(home_pose)  # 送出 Home 姿態。
                last_sent_pose = home_pose  # 更新最後姿態。

            elif key == ord('q'):  # 如果按下 q。
                break  # 離開主迴圈。

    finally:  # 無論正常結束或例外中斷，都會執行這裡。
        esp.send_text("STOP")  # 結束前送 STOP，讓系統回安全狀態。
        vision.release()  # 釋放攝影機。
        esp.close()  # 關閉 socket。
        cv2.destroyAllWindows()  # 關閉所有 OpenCV 視窗。


if __name__ == "__main__":  # 只有直接執行這支檔案時才會進入主程式。
    main()  # 啟動主程式。
