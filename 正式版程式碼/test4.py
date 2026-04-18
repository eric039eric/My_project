import cv2          # type: ignore # 匯入 OpenCV 函式庫，用於影像擷取與處理
import numpy as np  # type: ignore # 匯入 NumPy，用於建立 HSV 顏色範圍陣列
import socket       # 匯入 socket，用於建立 Python 端 UDP 通訊介面
import time         # 匯入 time，用於計算夾取等待時間

# =========================================================
# ESP32 網路設定
# =========================================================
ESP_IP   = "192.168.137.50"  # ESP32 固定 IP 位址，需與 Arduino 程式中設定的 local_ip 完全一致
ESP_PORT = 8888               # ESP32 UDP 監聽的 Port 號碼，需與 Arduino 的 localPort 一致
SOCKET_TIMEOUT = 0.2          # UDP socket 接收逾時秒數（此程式主要送出命令，非接收用）

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 建立 UDP socket 物件
sock.settimeout(SOCKET_TIMEOUT)                          # 設定 socket 逾時時間

# =========================================================
# 攝影機設定
# =========================================================
CAMERA_INDEX = 1  # 攝影機編號，0 為預設內建攝影機，1 為外接 USB 攝影機
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)  # 開啟攝影機，Windows 下使用 DirectShow 後端

# =========================================================
# 系統控制參數
# =========================================================
AUTO_RUN             = True  # 自動夾取模式開關，True 時偵測到穩定目標會自動送出 PICK 命令
LOCK_FRAMES          = 12    # 穩定幀數門檻，目標需在夾取區內連續穩定多少幀才觸發夾取
PICK_WAIT_SECONDS    = 4.5   # 送出 PICK 命令後等待手臂完成動作的秒數
CLEAR_FRAMES_REQUIRED = 8    # 夾取完成後夾取區需連續多少幀無目標才允許下一次夾取

MIN_RADIUS = 12   # 有效目標的最小包圍圓半徑（像素），低於此值視為雜訊
MIN_AREA   = 250  # 有效目標的最小輪廓面積（像素平方），低於此值視為雜訊

TARGET_ZONE_HALF_W = 35  # 固定夾取區的水平半寬（像素），中心點左右各 35 像素
TARGET_ZONE_HALF_H = 35  # 固定夾取區的垂直半高（像素），中心點上下各 35 像素
TARGET_STABLE_DIST = 18  # 連續兩幀同一目標的位置容許偏移距離（像素），超過此值視為不穩定

# =========================================================
# HSV 顏色辨識範圍設定
# =========================================================
# 紅色在 HSV 色環中橫跨 0° 與 180° 兩端，因此需要兩組範圍合併
lower_red1 = np.array([0,   120,  70])   # 紅色範圍一下限（低色相端）
upper_red1 = np.array([10,  255, 255])   # 紅色範圍一上限

lower_red2 = np.array([170, 120,  70])   # 紅色範圍二下限（高色相端）
upper_red2 = np.array([180, 255, 255])   # 紅色範圍二上限

lower_green = np.array([40,  50,  50])   # 綠色 HSV 範圍下限
upper_green = np.array([80,  255, 255])  # 綠色 HSV 範圍上限

lower_blue  = np.array([100,  50,  50])  # 藍色 HSV 範圍下限
upper_blue  = np.array([130, 255, 255])  # 藍色 HSV 範圍上限

# =========================================================
# 狀態機全域變數
# =========================================================
state           = "SEARCH"  # 目前系統狀態，初始為搜尋模式（SEARCH / WAIT_PICK / WAIT_CLEAR）
stable_count    = 0         # 目標在夾取區內的連續穩定幀數計數器
pick_start_time = 0.0       # 記錄送出 PICK 命令的時間點，用於計算等待倒數
last_cmd        = "-"       # 記錄最後送出的命令字串，顯示於畫面上供除錯用
last_target     = None      # 記錄上一幀偵測到的目標資訊，用於穩定性比較
clear_count     = 0         # 夾取區連續無目標的幀數計數器

# =========================================================
# 啟動資訊顯示
# =========================================================
print("=" * 60)
print("ESP32 固定 IP 對應版：Fixed Cross Auto Color Pick")
print(f"ESP32 IP  : {ESP_IP}")      # 顯示目前設定的 ESP32 IP
print(f"ESP32 PORT: {ESP_PORT}")    # 顯示通訊 Port
print(f"CAMERA IDX: {CAMERA_INDEX}")  # 顯示攝影機編號
print(f"AUTO_RUN  : {AUTO_RUN}")    # 顯示自動模式狀態
print("=" * 60)
print("快捷鍵:")
print("  q       = 離開程式")
print("  s       = 切換 AUTO_RUN")
print("  h       = HOME（手臂回原點）")
print("  o       = OPEN（夾爪打開）")
print("  c       = CLOSE（夾爪關閉）")
print("  u       = STATUS（查詢 ESP32 狀態）")
print("  p       = PING（測試 ESP32 是否在線）")
print("  r/g/b   = 手動觸發紅/綠/藍夾取")
print("  1/2/3/4 = 手動測試單顆伺服馬達回原點")
print("  z/x     = Base 馬達（M1）左右微調")
print("=" * 60)


# =========================================================
# 函式定義
# =========================================================

def send_cmd(cmd: str):
    """透過 UDP 將控制命令字串傳送至 ESP32"""
    global last_cmd
    try:
        sock.sendto(cmd.encode("utf-8"), (ESP_IP, ESP_PORT))  # 將命令編碼為 UTF-8 並送出
        last_cmd = cmd                                         # 更新最後送出命令紀錄
        print("SEND ->", cmd)                                  # 在 terminal 顯示送出的命令
    except Exception as e:
        print(f"SEND FAILED -> {cmd} | {e}")                  # 若傳送失敗則顯示錯誤訊息


def preprocess_mask(mask):
    """對遮罩進行形態學處理，去除雜訊並強化目標輪廓"""
    mask = cv2.erode(mask,  None, iterations=2)   # 侵蝕：消除遮罩中的小型雜訊點
    mask = cv2.dilate(mask, None, iterations=2)   # 膨脹：恢復目標輪廓面積，補回被侵蝕的邊緣
    return mask


def find_target(mask, frame, color_name, draw_color):
    """
    在指定顏色遮罩中尋找最大有效輪廓，
    若符合最小半徑與面積條件則回傳目標資訊字典，否則回傳 None
    """
    mask = preprocess_mask(mask)  # 先對遮罩做形態學處理

    # 在遮罩中搜尋所有外部輪廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:   # 若沒有找到任何輪廓則直接回傳 None
        return None

    c    = max(contours, key=cv2.contourArea)  # 取面積最大的輪廓作為候選目標
    area = cv2.contourArea(c)                  # 計算該輪廓的像素面積

    if area < MIN_AREA:   # 若面積小於最小門檻則視為雜訊，回傳 None
        return None

    (center, radius) = cv2.minEnclosingCircle(c)  # 計算輪廓的最小包圍圓，取得圓心與半徑
    cx, cy = center                               # 解包圓心座標

    if radius < MIN_RADIUS:   # 若半徑小於最小門檻則視為雜訊，回傳 None
        return None

    # 在畫面上繪製目標的包圍圓
    cv2.circle(frame, (int(cx), int(cy)), int(radius), draw_color, 2)

    # 在目標圓心附近標註顏色名稱
    cv2.putText(
        frame, color_name,
        (int(cx) - 20, int(cy) - 18),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2,
    )

    # 回傳目標資訊字典，供後續判斷與選擇使用
    return {
        "color" : color_name,       # 目標顏色名稱（RED / GREEN / BLUE）
        "cx"    : int(cx),          # 目標圓心 X 座標（像素）
        "cy"    : int(cy),          # 目標圓心 Y 座標（像素）
        "radius": float(radius),    # 目標包圍圓半徑
        "area"  : float(area),      # 目標輪廓面積
    }


def choose_target(targets, center_x, center_y):
    """
    從多個候選目標中選出最適合夾取的目標，
    評分依據為目標與畫面中心的距離（越近越好）並加入面積加分（越大越好）
    """
    valid = [t for t in targets if t is not None]  # 過濾掉 None，保留有效目標
    if not valid:
        return None  # 若無有效目標則回傳 None

    def score(t):
        dist       = abs(t["cx"] - center_x) + abs(t["cy"] - center_y)  # 計算目標與中心的曼哈頓距離
        area_bonus = t["area"] * 0.002                                    # 面積越大給予越高的扣分補償（降低評分值）
        return dist - area_bonus                                          # 評分值越小代表越優先

    return min(valid, key=score)  # 回傳評分最低（最優先）的目標


def update_stable_count(target):
    """
    根據本幀目標與上一幀目標的顏色與位置比較，
    判斷目標是否持續穩定，並更新穩定幀數計數器
    """
    global stable_count, last_target

    if target is None:           # 若本幀無目標則重置所有穩定紀錄
        stable_count = 0
        last_target  = None
        return

    if last_target is None:      # 若上一幀無目標（剛開始偵測），從 1 開始計數
        stable_count = 1
    else:
        # 判斷顏色是否與上一幀相同
        same_color   = target["color"] == last_target["color"]

        # 判斷位置偏移是否在允許範圍內（X 與 Y 方向都需符合）
        close_enough = (
            abs(target["cx"] - last_target["cx"]) <= TARGET_STABLE_DIST
            and abs(target["cy"] - last_target["cy"]) <= TARGET_STABLE_DIST
        )

        if same_color and close_enough:
            stable_count += 1   # 顏色相同且位置穩定，累加穩定幀數
        else:
            stable_count = 1    # 顏色改變或位置跳動，重新從 1 開始計數

    # 更新上一幀目標資訊（只保留判斷所需的欄位）
    last_target = {
        "color": target["color"],
        "cx"   : target["cx"],
        "cy"   : target["cy"],
    }


def is_target_in_pick_zone(target, center_x, center_y):
    """
    判斷目標是否位於畫面中心的固定夾取區矩形範圍內，
    回傳 True 表示目標在區域內，可進行穩定幀數累積
    """
    if target is None:
        return False  # 無目標直接回傳 False

    error_x = target["cx"] - center_x   # 目標中心與畫面中心的水平偏差
    error_y = target["cy"] - center_y   # 目標中心與畫面中心的垂直偏差

    # 水平與垂直偏差均在允許範圍內才視為在夾取區
    return abs(error_x) <= TARGET_ZONE_HALF_W and abs(error_y) <= TARGET_ZONE_HALF_H


def reset_search_state():
    """將狀態機重置回 SEARCH 狀態，並清除所有計數器與上一幀目標紀錄"""
    global state, stable_count, last_target, clear_count
    state        = "SEARCH"
    stable_count = 0
    last_target  = None
    clear_count  = 0


# =========================================================
# 攝影機開啟驗證
# =========================================================
if not cap.isOpened():
    print("無法開啟攝影機，請檢查 CAMERA_INDEX。")
    raise SystemExit  # 若攝影機無法開啟則直接終止程式


# =========================================================
# 主迴圈：每幀執行一次影像處理、狀態判斷與畫面顯示
# =========================================================
while True:
    ret, frame = cap.read()   # 從攝影機讀取一幀畫面，ret 為是否成功的布林值
    if not ret:
        print("讀不到攝影機畫面，程式結束。")
        break  # 若讀取失敗則跳出迴圈，結束程式

    h, w, _ = frame.shape    # 取得畫面的高度（h）與寬度（w）
    center_x = w // 2        # 計算畫面水平中心座標
    center_y = h // 2        # 計算畫面垂直中心座標

    # ── 繪製十字準線 ──────────────────────────────────────
    cv2.line(frame, (center_x, 0), (center_x, h), (255, 255, 255), 1)  # 垂直白色準線
    cv2.line(frame, (0, center_y), (w, center_y), (255, 255, 255), 1)  # 水平白色準線

    # ── 計算並繪製固定夾取區矩形 ──────────────────────────
    box_left   = center_x - TARGET_ZONE_HALF_W   # 夾取區左邊界 X
    box_right  = center_x + TARGET_ZONE_HALF_W   # 夾取區右邊界 X
    box_top    = center_y - TARGET_ZONE_HALF_H   # 夾取區上邊界 Y
    box_bottom = center_y + TARGET_ZONE_HALF_H   # 夾取區下邊界 Y

    cv2.rectangle(frame, (box_left, box_top), (box_right, box_bottom), (0, 255, 255), 2)  # 畫黃色矩形框
    cv2.putText(
        frame, "FIXED PICK ZONE",
        (box_left - 10, box_top - 8),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2,
    )  # 在矩形框上方標示文字說明

    # ── 影像前處理 ────────────────────────────────────────
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)          # 高斯模糊，降低雜訊與反光干擾
    hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)       # 將模糊後的影像從 BGR 轉換為 HSV 色彩空間

    # ── 建立各顏色遮罩 ────────────────────────────────────
    mask_r = (cv2.inRange(hsv, lower_red1, upper_red1) +
              cv2.inRange(hsv, lower_red2, upper_red2))   # 合併紅色兩段範圍遮罩（低色相 + 高色相）
    mask_g = cv2.inRange(hsv, lower_green, upper_green)   # 建立綠色遮罩
    mask_b = cv2.inRange(hsv, lower_blue,  upper_blue)    # 建立藍色遮罩

    # ── 各顏色目標偵測 ────────────────────────────────────
    target_r = find_target(mask_r, frame, "RED",   (0,   0, 255))  # 在紅色遮罩中尋找目標，繪製紅色圓圈
    target_g = find_target(mask_g, frame, "GREEN", (0, 255,   0))  # 在綠色遮罩中尋找目標，繪製綠色圓圈
    target_b = find_target(mask_b, frame, "BLUE",  (255, 0,   0))  # 在藍色遮罩中尋找目標，繪製藍色圓圈

    # 從三個候選目標中選出最接近中心且面積最大的作為主要目標
    target = choose_target([target_r, target_g, target_b], center_x, center_y)

    # ── 初始化畫面顯示資訊 ────────────────────────────────
    status_msg = "SEARCH"   # 狀態訊息，預設為 SEARCH
    color_msg  = "-"        # 顏色訊息，預設為無目標
    zone_msg   = "OUT"      # 區域訊息，預設目標不在夾取區

    in_pick_zone = is_target_in_pick_zone(target, center_x, center_y)  # 判斷目標是否在固定夾取區內

    if target is not None:
        color_msg = target["color"]                       # 更新顏色訊息
        zone_msg  = "IN" if in_pick_zone else "OUT"       # 更新區域訊息

    # =========================================================
    # 狀態機邏輯
    # =========================================================

    # ── 狀態：WAIT_PICK（等待手臂完成夾取動作）──────────────
    if state == "WAIT_PICK":
        remain = PICK_WAIT_SECONDS - (time.time() - pick_start_time)  # 計算剩餘等待秒數
        if remain <= 0:
            # 等待時間到，轉換至 WAIT_CLEAR 狀態，重置穩定相關計數器
            state        = "WAIT_CLEAR"
            clear_count  = 0
            stable_count = 0
            last_target  = None
        else:
            status_msg = f"WAIT_PICK {remain:.1f}s"  # 顯示剩餘等待時間

    # ── 狀態：WAIT_CLEAR（等待夾取區完全清空）────────────────
    elif state == "WAIT_CLEAR":
        if in_pick_zone:
            clear_count = 0                            # 若夾取區仍有目標，重置清空計數器
            status_msg  = "WAIT_REMOVE_OBJECT"         # 提示使用者移開物件
        else:
            clear_count += 1                           # 夾取區無目標，累加清空幀數
            status_msg   = f"WAIT_CLEAR {clear_count}/{CLEAR_FRAMES_REQUIRED}"
            if clear_count >= CLEAR_FRAMES_REQUIRED:
                # 清空幀數達門檻，確認物件已移離，重置狀態回 SEARCH
                state        = "SEARCH"
                clear_count  = 0
                stable_count = 0
                last_target  = None
                status_msg   = "READY_NEXT_OBJECT"

    # ── 狀態：SEARCH（搜尋並鎖定目標）───────────────────────
    elif state == "SEARCH":
        if target is None:
            stable_count = 0       # 無目標時重置穩定計數
            last_target  = None
            status_msg   = "SEARCH -> No target"
        else:
            if in_pick_zone:
                update_stable_count(target)                            # 目標在區域內，更新穩定幀數
                status_msg = f"LOCKING {stable_count}/{LOCK_FRAMES}"  # 顯示鎖定進度
                cv2.circle(frame, (target["cx"], target["cy"]), 6, (0, 255, 255), -1)  # 在目標中心畫實心點
            else:
                stable_count = 0      # 目標不在夾取區，重置穩定計數
                last_target  = None
                status_msg   = "Place object on cross"  # 提示使用者將物件移至中心區

        # 若目標在夾取區、穩定幀數達門檻、且自動模式開啟，則送出夾取命令
        if in_pick_zone and stable_count >= LOCK_FRAMES and AUTO_RUN:
            cmd = f"PICK:{target['color']}"          # 組合對應顏色的夾取命令字串
            print(f"Detected color: {target['color']}")
            send_cmd(cmd)                            # 透過 UDP 送出命令至 ESP32
            pick_start_time = time.time()            # 記錄命令送出的時間點
            state        = "WAIT_PICK"               # 切換狀態為等待手臂完成
            stable_count = 0                         # 重置穩定幀數
            last_target  = None                      # 清除上一幀目標紀錄
            clear_count  = 0                         # 重置清空計數器
            status_msg   = f"SEND {cmd}"             # 更新狀態訊息

    # =========================================================
    # 畫面資訊顯示（左上角 HUD）
    # =========================================================
    cv2.putText(frame, f"ESP32 IP: {ESP_IP}",              (10, 30),  cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255,   0), 2)  # 顯示 ESP32 IP
    cv2.putText(frame, f"State: {status_msg}",             (10, 60),  cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2)  # 顯示目前狀態
    cv2.putText(frame, f"Color: {color_msg}",              (10, 90),  cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2)  # 顯示偵測顏色
    cv2.putText(frame, f"Zone: {zone_msg}",                (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.68,
                (0, 255, 0) if zone_msg == "IN" else (0, 0, 255), 2)                                                        # 在區域內顯示綠色，否則紅色
    cv2.putText(frame, f"Stable: {stable_count}/{LOCK_FRAMES}",       (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)  # 顯示穩定幀數進度
    cv2.putText(frame, f"Clear: {clear_count}/{CLEAR_FRAMES_REQUIRED}",(10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)  # 顯示清空幀數進度
    cv2.putText(frame, f"AUTO_RUN: {AUTO_RUN}",            (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255,   0), 2)  # 顯示自動模式狀態
    cv2.putText(frame, f"LAST CMD: {last_cmd}",            (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,  255, 255), 2)  # 顯示最後送出的命令

    cv2.imshow("Fixed Cross Auto Color Pick - Static IP", frame)  # 將處理後的畫面顯示在視窗中

    # =========================================================
    # 鍵盤控制
    # =========================================================
    key = cv2.waitKey(1) & 0xFF  # 等待 1ms 鍵盤輸入，取低 8 位元

    if key == ord("q"):
        break   # 按 q 離開主迴圈，結束程式

    elif key == ord("s"):
        AUTO_RUN = not AUTO_RUN   # 切換自動模式開關
        stable_count = 0
        last_target  = None
        clear_count  = 0
        if state != "WAIT_PICK":   # 若不在等待手臂期間，重置狀態回 SEARCH
            state = "SEARCH"
        print("AUTO_RUN =", AUTO_RUN)

    elif key == ord("h"):
        send_cmd("HOME")    # 送出 HOME 命令，手臂回到初始姿態

    elif key == ord("o"):
        send_cmd("OPEN")    # 送出 OPEN 命令，夾爪打開

    elif key == ord("c"):
        send_cmd("CLOSE")   # 送出 CLOSE 命令，夾爪關閉

    elif key == ord("u"):
        send_cmd("STATUS")  # 送出 STATUS 命令，查詢 ESP32 目前狀態

    elif key == ord("p"):
        send_cmd("PING")    # 送出 PING 命令，測試 ESP32 是否在線（應回傳 PONG）

    elif key == ord("r"):
        send_cmd("PICK:RED")    # 手動觸發紅色積木夾取流程

    elif key == ord("g"):
        send_cmd("PICK:GREEN")  # 手動觸發綠色積木夾取流程

    elif key == ord("b"):
        send_cmd("PICK:BLUE")   # 手動觸發藍色積木夾取流程

    elif key == ord("1"):
        send_cmd("M1:90")   # 手動將 M1（底座）伺服馬達移至 90 度

    elif key == ord("2"):
        send_cmd("M2:90")   # 手動將 M2（主臂）伺服馬達移至 90 度

    elif key == ord("3"):
        send_cmd("M3:90")   # 手動將 M3（輔臂）伺服馬達移至 90 度

    elif key == ord("4"):
        send_cmd("M4:110")  # 手動將 M4（夾爪）伺服馬達移至 110 度（半開狀態）

    elif key == ord("z"):
        send_cmd("BASE:80")   # Base 馬達往左微調至 80 度

    elif key == ord("x"):
        send_cmd("BASE:100")  # Base 馬達往右微調至 100 度

# =========================================================
# 程式結束，釋放資源
# =========================================================
cap.release()           # 釋放攝影機資源
cv2.destroyAllWindows() # 關閉所有 OpenCV 視窗
sock.close()            # 關閉 UDP socket
