import cv2 # type: ignore
import numpy as np # type: ignore
import socket
import time

# ================= 網路設定 =================
ESP_IP = "192.168.137.170"
ESP_PORT = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ================= 攝影機設定 =================
CAMERA_INDEX = 1
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

# ================= 模式設定 =================
AUTO_PICK = True   # True: 鎖定後自動送 PICK:顏色
                   # False: 只追蹤底座，不自動夾取

# ================= 參數設定 =================
CENTER_TOLERANCE = 25
LOCK_FRAMES = 12
MIN_RADIUS = 12
SEND_ANGLE_DELTA = 4
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

# ================= 狀態 =================
state = "SEARCH"
stable_count = 0
last_sent_angle = None
last_send_time = 0.0
last_no_target_time = 0.0
pick_start_time = 0.0

print(f"ESP32 IP: {ESP_IP}")
print("快捷鍵: q=離開, h=HOME, o=OPEN, c=CLOSE, r/g/b=直接送 PICK")
print(f"AUTO_PICK = {AUTO_PICK}")

def send_cmd(cmd: str):
    sock.sendto(cmd.encode("utf-8"), (ESP_IP, ESP_PORT))
    print("SEND ->", cmd)

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
    ((cx, cy), radius) = cv2.minEnclosingCircle(c)

    if radius < MIN_RADIUS:
        return None

    area = cv2.contourArea(c)
    angle = int(np.interp(cx, [0, width], [180, 0]))

    cv2.circle(frame, (int(cx), int(cy)), int(radius), draw_color, 2)
    cv2.putText(
        frame,
        f"{color_name}: {angle} deg",
        (int(cx) - 30, int(cy) - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
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
    err_msg = "-"

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
            send_center_if_needed()
        else:
            color_msg = target["color"]
            angle_msg = str(target["angle"])
            error_x = target["cx"] - center_x
            err_msg = str(error_x)

            state = "ALIGN"
            send_base_if_needed(target["angle"])

            if abs(error_x) <= CENTER_TOLERANCE:
                stable_count += 1
                status_msg = f"LOCKING {stable_count}/{LOCK_FRAMES}"
            else:
                stable_count = 0
                status_msg = f"ALIGN {target['color']}"

            if stable_count >= LOCK_FRAMES:
                if AUTO_PICK:
                    cmd = f"PICK:{target['color']}"
                    send_cmd(cmd)
                    pick_start_time = time.time()
                    state = "WAIT_PICK"
                    stable_count = 0
                    status_msg = f"SEND {cmd}"
                else:
                    status_msg = "LOCKED (AUTO_PICK=False)"

    cv2.line(frame, (center_x, 0), (center_x, h), (255, 255, 255), 1)
    cv2.putText(frame, f"State: {status_msg}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"Color: {color_msg}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"Angle: {angle_msg}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"ErrX: {err_msg}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.imshow("Auto Pick Controller", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break
    elif key == ord("h"):
        send_cmd("HOME")
    elif key == ord("o"):
        send_cmd("OPEN")
    elif key == ord("c"):
        send_cmd("CLOSE")
    elif key == ord("r"):
        send_cmd("PICK:RED")
    elif key == ord("g"):
        send_cmd("PICK:GREEN")
    elif key == ord("b"):
        send_cmd("PICK:BLUE")

cap.release()
cv2.destroyAllWindows()
