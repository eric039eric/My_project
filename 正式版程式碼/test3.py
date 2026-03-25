import cv2  # type: ignore
import numpy as np  # type: ignore
import socket
import time

# ================= 網路設定 =================
ESP_IP = "192.168.137.72"   # 改成 ESP32 序列埠顯示的 IP
ESP_PORT = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ================= 攝影機設定 =================
CAMERA_INDEX = 1
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

# ================= 模式設定 =================
AUTO_PICK = False
BASE_ENABLE = False
ONE_SHOT_BASE = False

# ================= 手動角度狀態 =================
manual_angles = {
    1: 90,
    2: 90,
    3: 90,
    4: 110,
}
STEP = 2

# ================= 視覺參數 =================
MIN_RADIUS = 12
MIN_AREA = 250
TARGET_ZONE_HALF_W = 35
TARGET_ZONE_HALF_H = 35
LOCK_FRAMES = 12
SEND_ANGLE_DELTA = 3
SEND_INTERVAL = 0.08
NO_TARGET_INTERVAL = 0.5
PICK_WAIT_SECONDS = 4.5

# ================= HSV 範圍 =================
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])

lower_green = np.array([40, 50, 50])
upper_green = np.array([80, 255, 255])

lower_blue = np.array([100, 50, 50])
upper_blue = np.array([130, 255, 255])

# ================= 狀態變數 =================
state = "SEARCH"
stable_count = 0
last_sent_angle = None
last_send_time = 0.0
last_no_target_time = 0.0
pick_start_time = 0.0
last_cmd = "-"

print(f"ESP32 IP: {ESP_IP}")
print("快捷鍵:")
print(" q = 離開")
print(" h = HOME")
print(" o = OPEN")
print(" c = CLOSE")
print(" i = PING")
print(" u = STATUS")
print(" t = 切換連續 BASE 追蹤")
print(" m = 只送一次目前目標的 BASE")
print(" p = 切換 AUTO_PICK")
print(" r / g / b = 直接送 PICK")
print(" 1/2 = M1 -/+")
print(" 3/4 = M2 -/+")
print(" 5/6 = M3 -/+")
print(" 7/8 = M4 -/+")

def send_cmd(cmd: str):
    global last_cmd
    sock.sendto(cmd.encode("utf-8"), (ESP_IP, ESP_PORT))
    last_cmd = cmd
    print("SEND ->", cmd)

def send_motor(motor_id: int, angle: int):
    angle = int(np.clip(angle, 0, 180))
    manual_angles[motor_id] = angle
    send_cmd(f"M{motor_id}:{angle}")

def adjust_motor(motor_id: int, delta: int):
    send_motor(motor_id, manual_angles[motor_id] + delta)

def send_base_if_needed(angle: int):
    global last_sent_angle, last_send_time
    now = time.time()
    angle = int(np.clip(angle, 0, 180))

    if last_sent_angle is None:
        send_cmd(f"BASE:{angle}")
        last_sent_angle = angle
        last_send_time = now
        return

    if abs(angle - last_sent_angle) >= SEND_ANGLE_DELTA and (now - last_send_time) >= SEND_INTERVAL:
        send_cmd(f"BASE:{angle}")
        last_sent_angle = angle
        last_send_time = now

def send_center_if_needed():
    global last_sent_angle, last_send_time, last_no_target_time
    now = time.time()
    if now - last_no_target_time >= NO_TARGET_INTERVAL:
        send_cmd("BASE:90")
        last_sent_angle = 90
        last_send_time = now
        last_no_target_time = now

def preprocess_mask(mask):
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    return mask

def find_target(mask, frame, width, color_name, draw_color):
    mask = preprocess_mask(mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return None

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_AREA:
        return None

    ((cx, cy), radius) = cv2.minEnclosingCircle(c)
    if radius < MIN_RADIUS:
        return None

    angle = int(np.interp(cx, [0, width], [180, 0]))

    cv2.circle(frame, (int(cx), int(cy)), int(radius), draw_color, 2)
    cv2.putText(
        frame,
        f"{color_name}: {angle} deg",
        (int(cx) - 35, int(cy) - 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        draw_color,
        2,
    )

    return {
        "color": color_name,
        "cx": int(cx),
        "cy": int(cy),
        "radius": float(radius),
        "area": float(area),
        "angle": angle,
    }

def choose_target(r, g, b):
    if r is not None:
        return r
    if g is not None:
        return g
    if b is not None:
        return b
    return None

while True:
    ret, frame = cap.read()
    if not ret:
        print("讀不到攝影機畫面，程式結束。")
        break

    h, w, _ = frame.shape
    center_x = w // 2
    center_y = h // 2

    cv2.line(frame, (center_x, 0), (center_x, h), (255, 255, 255), 1)
    cv2.line(frame, (0, center_y), (w, center_y), (255, 255, 255), 1)

    box_left = center_x - TARGET_ZONE_HALF_W
    box_right = center_x + TARGET_ZONE_HALF_W
    box_top = center_y - TARGET_ZONE_HALF_H
    box_bottom = center_y + TARGET_ZONE_HALF_H
    cv2.rectangle(frame, (box_left, box_top), (box_right, box_bottom), (0, 255, 255), 2)
    cv2.putText(frame, "PICK ZONE", (box_left, box_top - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask_r = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    mask_g = cv2.inRange(hsv, lower_green, upper_green)
    mask_b = cv2.inRange(hsv, lower_blue, upper_blue)

    target_r = find_target(mask_r, frame, w, "RED", (0, 0, 255))
    target_g = find_target(mask_g, frame, w, "GREEN", (0, 255, 0))
    target_b = find_target(mask_b, frame, w, "BLUE", (255, 0, 0))
    target = choose_target(target_r, target_g, target_b)

    status_msg = "Idle"
    color_msg = "-"
    angle_msg = "-"
    errx_msg = "-"
    erry_msg = "-"
    zone_msg = "OUT"

    if state == "WAIT_PICK":
        remain = PICK_WAIT_SECONDS - (time.time() - pick_start_time)
        if remain <= 0:
            state = "SEARCH"
            stable_count = 0
        else:
            status_msg = f"WAIT_PICK {remain:.1f}s"

    if state != "WAIT_PICK":
        if target is None:
            state = "SEARCH"
            stable_count = 0
            status_msg = "SEARCH -> No target"

            if BASE_ENABLE:
                send_center_if_needed()

            if ONE_SHOT_BASE:
                print("ONE_SHOT_BASE ignored: no target")
                ONE_SHOT_BASE = False

        else:
            color_msg = target["color"]
            angle_msg = str(target["angle"])

            error_x = target["cx"] - center_x
            error_y = target["cy"] - center_y
            errx_msg = str(error_x)
            erry_msg = str(error_y)

            in_pick_zone = (
                abs(error_x) <= TARGET_ZONE_HALF_W and
                abs(error_y) <= TARGET_ZONE_HALF_H
            )
            zone_msg = "IN" if in_pick_zone else "OUT"
            state = "ALIGN"

            if BASE_ENABLE:
                send_base_if_needed(target["angle"])
            elif ONE_SHOT_BASE:
                send_cmd(f"BASE:{target['angle']}")
                last_sent_angle = target["angle"]
                last_send_time = time.time()
                ONE_SHOT_BASE = False

            if in_pick_zone:
                stable_count += 1
                status_msg = f"LOCKING {stable_count}/{LOCK_FRAMES}"
                cv2.circle(frame, (target["cx"], target["cy"]), 6, (0, 255, 255), -1)
            else:
                stable_count = 0
                status_msg = f"ALIGN {target['color']}"

            if in_pick_zone and stable_count >= LOCK_FRAMES:
                if AUTO_PICK:
                    cmd = f"PICK:{target['color']}"
                    send_cmd(cmd)
                    pick_start_time = time.time()
                    state = "WAIT_PICK"
                    stable_count = 0
                    status_msg = f"SEND {cmd}"
                else:
                    status_msg = "LOCKED (AUTO_PICK=False)"

    cv2.putText(frame, f"State: {status_msg}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2)
    cv2.putText(frame, f"Color: {color_msg}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2)
    cv2.putText(frame, f"Angle: {angle_msg}", (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2)
    cv2.putText(frame, f"ErrX: {errx_msg}", (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2)
    cv2.putText(frame, f"ErrY: {erry_msg}", (10, 150),
                cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2)
    cv2.putText(frame, f"Zone: {zone_msg}", (10, 180),
                cv2.FONT_HERSHEY_SIMPLEX, 0.68,
                (0, 255, 0) if zone_msg == "IN" else (0, 0, 255), 2)
    cv2.putText(frame, f"BASE_ENABLE: {BASE_ENABLE}", (10, 210),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 0), 2)
    cv2.putText(frame, f"AUTO_PICK: {AUTO_PICK}", (10, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 0), 2)
    cv2.putText(frame, f"LAST CMD: {last_cmd}", (10, 270),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    cv2.imshow("Center Lock Auto Pick", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break
    elif key == ord("h"):
        send_cmd("HOME")
    elif key == ord("o"):
        send_cmd("OPEN")
    elif key == ord("c"):
        send_cmd("CLOSE")
    elif key == ord("i"):
        send_cmd("PING")
    elif key == ord("u"):
        send_cmd("STATUS")
    elif key == ord("r"):
        send_cmd("PICK:RED")
    elif key == ord("g"):
        send_cmd("PICK:GREEN")
    elif key == ord("b"):
        send_cmd("PICK:BLUE")
    elif key == ord("t"):
        BASE_ENABLE = not BASE_ENABLE
        print("BASE_ENABLE =", BASE_ENABLE)
    elif key == ord("m"):
        ONE_SHOT_BASE = True
        print("ONE_SHOT_BASE = True")
    elif key == ord("p"):
        AUTO_PICK = not AUTO_PICK
        print("AUTO_PICK =", AUTO_PICK)
    elif key == ord("1"):
        adjust_motor(1, -STEP)
    elif key == ord("2"):
        adjust_motor(1, STEP)
    elif key == ord("3"):
        adjust_motor(2, -STEP)
    elif key == ord("4"):
        adjust_motor(2, STEP)
    elif key == ord("5"):
        adjust_motor(3, -STEP)
    elif key == ord("6"):
        adjust_motor(3, STEP)
    elif key == ord("7"):
        adjust_motor(4, -STEP)
    elif key == ord("8"):
        adjust_motor(4, STEP)

cap.release()
cv2.destroyAllWindows()
