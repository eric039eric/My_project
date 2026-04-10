import cv2  # type: ignore # 匯入 OpenCV，用來開啟攝影機、做影像處理、畫畫面提示
import numpy as np  # type: ignore # 匯入 NumPy，用來建立 HSV 顏色範圍陣列與做一些數值運算
import socket  # 匯入 socket，讓 Python 可以透過 UDP 傳送指令給 ESP32
import time  # 匯入 time，讓程式可以記錄時間與等待夾取完成

ESP_IP = "192.168.137.196"  # 設定 ESP32 的 IP 位址，這裡要改成你自己 ESP32 實際連線後顯示的 IP
ESP_PORT = 8888  # 設定 ESP32 接收 UDP 指令的 Port
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 建立一個 UDP socket，之後所有文字命令都由它送出

CAMERA_INDEX = 1  # 設定要使用哪一台攝影機，如果你的電腦只有一台鏡頭可試 0
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)  # 用 DirectShow 模式開啟攝影機，Windows 上通常比較穩定

AUTO_RUN = True  # 設定是否啟用全自動模式，True 代表辨識成功後會自動送出夾取指令
LOCK_FRAMES = 12  # 設定目標必須連續穩定幾幀才會觸發夾取，避免誤判
PICK_WAIT_SECONDS = 4.5  # 設定送出 PICK 指令後等待幾秒，讓手臂有時間完成整個夾取流程
CLEAR_FRAMES_REQUIRED = 8  # 設定完成一輪後，十字區域必須連續空白幾幀，才允許下一次夾取
MIN_RADIUS = 12  # 設定最小包圍圓半徑，小於此值的區塊通常視為雜訊
MIN_AREA = 250  # 設定最小輪廓面積，小於此值的顏色區塊通常視為雜訊
TARGET_ZONE_HALF_W = 35  # 設定中心夾取區的一半寬度，物體中心在這範圍內才算放在十字點位附近
TARGET_ZONE_HALF_H = 35  # 設定中心夾取區的一半高度，物體中心在這範圍內才算放在十字點位附近
TARGET_STABLE_DIST = 18  # 設定兩幀之間若目標位移小於這個像素值，就視為同一個穩定目標

lower_red1 = np.array([0, 120, 70])  # 設定紅色第一段 HSV 下界，因為紅色會跨越 HSV 的起點
upper_red1 = np.array([10, 255, 255])  # 設定紅色第一段 HSV 上界
lower_red2 = np.array([170, 120, 70])  # 設定紅色第二段 HSV 下界，用來補另一側紅色範圍
upper_red2 = np.array([180, 255, 255])  # 設定紅色第二段 HSV 上界
lower_green = np.array([40, 50, 50])  # 設定綠色 HSV 下界
upper_green = np.array([80, 255, 255])  # 設定綠色 HSV 上界
lower_blue = np.array([100, 50, 50])  # 設定藍色 HSV 下界
upper_blue = np.array([130, 255, 255])  # 設定藍色 HSV 上界

state = "SEARCH"  # 建立狀態機初始狀態，SEARCH 代表正在等待物體出現在中心區
stable_count = 0  # 建立穩定幀數計數器，一開始先設為 0
pick_start_time = 0.0  # 建立夾取開始時間變數，一開始先設為 0
last_cmd = "-"  # 建立最後送出命令的顯示字串，初始先顯示 -
last_target = None  # 建立上一幀目標資訊，用來判斷目前是不是同一個穩定目標
clear_count = 0  # 建立清空幀數計數器，用來確認夾取後十字範圍已經真的沒有物體

print(f"ESP32 IP: {ESP_IP}")  # 在終端機印出目前要連線的 ESP32 IP，方便檢查
print("固定十字點位單次自動辨色夾取版")  # 印出目前版本名稱，提醒這一版是一物一抓版本
print("快捷鍵:")  # 印出操作說明標題
print(" q = 離開程式")  # 提示按 q 可以結束程式
print(" s = 切換 AUTO_RUN")  # 提示按 s 可以切換自動模式開或關
print(" h = HOME")  # 提示按 h 可以要求手臂回 HOME
print(" o = OPEN")  # 提示按 o 可以打開夾爪
print(" c = CLOSE")  # 提示按 c 可以關閉夾爪
print(" u = STATUS")  # 提示按 u 可以要求 ESP32 回報狀態
print(" r / g / b = 手動測試顏色夾取")  # 提示按 r、g、b 可以直接測試指定顏色夾取
print(f"啟動狀態: AUTO_RUN={AUTO_RUN}")  # 印出程式啟動時自動模式的狀態


def send_cmd(cmd: str):  # 定義 send_cmd 函式，負責統一送出 UDP 指令到 ESP32
    global last_cmd  # 宣告要修改全域變數 last_cmd
    sock.sendto(cmd.encode("utf-8"), (ESP_IP, ESP_PORT))  # 把文字命令轉成 UTF-8 後透過 UDP 傳給 ESP32
    last_cmd = cmd  # 把本次送出的命令記錄起來，方便顯示在畫面上
    print("SEND ->", cmd)  # 在終端機印出送出的命令，方便除錯與展示



def preprocess_mask(mask):  # 定義遮罩前處理函式，讓顏色區塊更乾淨
    mask = cv2.erode(mask, None, iterations=2)  # 先做侵蝕，去掉小雜點
    mask = cv2.dilate(mask, None, iterations=2)  # 再做膨脹，把真正目標補回來
    return mask  # 回傳處理後的遮罩



def find_target(mask, frame, color_name, draw_color):  # 定義找單一顏色目標的函式
    mask = preprocess_mask(mask)  # 先把遮罩做基本去雜訊處理
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 找出遮罩中的外部輪廓

    if len(contours) == 0:  # 如果完全沒有輪廓
        return None  # 直接回傳 None，代表沒有找到這個顏色的物體

    c = max(contours, key=cv2.contourArea)  # 從所有輪廓中選出面積最大的那一個
    area = cv2.contourArea(c)  # 計算這個最大輪廓的面積
    if area < MIN_AREA:  # 如果輪廓面積小於門檻
        return None  # 視為雜訊，不當成有效目標

    (center, radius) = cv2.minEnclosingCircle(c)  # 找出能包住該輪廓的最小外接圓
    cx, cy = center  # 把圓心座標拆成 cx 與 cy
    if radius < MIN_RADIUS:  # 如果圓半徑小於門檻
        return None  # 視為雜訊，不當成有效目標

    cv2.circle(frame, (int(cx), int(cy)), int(radius), draw_color, 2)  # 在畫面上畫出該物體的外接圓
    cv2.putText(frame, f"{color_name}", (int(cx) - 20, int(cy) - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)  # 在物體旁邊寫上辨識到的顏色名稱

    return {  # 用字典格式回傳這個目標的重要資訊
        "color": color_name,  # 回傳目標顏色名稱
        "cx": int(cx),  # 回傳目標中心 x 座標
        "cy": int(cy),  # 回傳目標中心 y 座標
        "radius": float(radius),  # 回傳目標外接圓半徑
        "area": float(area),  # 回傳目標輪廓面積
    }  # 結束字典回傳



def choose_target(targets, center_x, center_y):  # 定義挑選目標函式，當多種顏色同時存在時選一個最適合的
    valid = [t for t in targets if t is not None]  # 先把非 None 的有效目標挑出來
    if not valid:  # 如果完全沒有有效目標
        return None  # 回傳 None

    def score(t):  # 定義一個內部評分函式
        dist = abs(t["cx"] - center_x) + abs(t["cy"] - center_y)  # 計算目標到畫面中心的距離
        area_bonus = t["area"] * 0.002  # 給面積大的目標一些加分，代表較穩定易辨識
        return dist - area_bonus  # 距離越小越好、面積越大越好，所以回傳距離減去面積加分

    return min(valid, key=score)  # 從有效目標中選出分數最低的那個當成主要目標



def update_stable_count(target):  # 定義穩定計數函式，用來避免目標抖動時誤觸發
    global stable_count  # 宣告會修改全域變數 stable_count
    global last_target  # 宣告會修改全域變數 last_target

    if target is None:  # 如果目前沒有目標
        stable_count = 0  # 把穩定計數歸零
        last_target = None  # 把上一幀目標清空
        return  # 直接離開函式

    if last_target is None:  # 如果上一幀還沒有目標
        stable_count = 1  # 代表從第 1 幀開始累計
    else:  # 如果上一幀有目標
        same_color = target["color"] == last_target["color"]  # 判斷這一幀與上一幀顏色是否相同
        close_enough = abs(target["cx"] - last_target["cx"]) <= TARGET_STABLE_DIST and abs(target["cy"] - last_target["cy"]) <= TARGET_STABLE_DIST  # 判斷兩幀之間位置是否足夠接近
        if same_color and close_enough:  # 如果顏色相同而且位置差距也不大
            stable_count += 1  # 把穩定計數加一
        else:  # 如果顏色變了或位置跳太多
            stable_count = 1  # 把這一幀當成新目標重新開始計數

    last_target = {"color": target["color"], "cx": target["cx"], "cy": target["cy"]}  # 把這一幀的目標記下來給下一幀比較



def is_target_in_pick_zone(target, center_x, center_y):  # 定義判斷函式，用來檢查目前目標是否在十字中心夾取區內
    if target is None:  # 如果沒有目標
        return False  # 直接回傳 False
    error_x = target["cx"] - center_x  # 計算目標在 x 方向相對中心的誤差
    error_y = target["cy"] - center_y  # 計算目標在 y 方向相對中心的誤差
    return abs(error_x) <= TARGET_ZONE_HALF_W and abs(error_y) <= TARGET_ZONE_HALF_H  # 只有 x 與 y 都在容許範圍內才算在夾取區中


while True:  # 進入主迴圈，讓程式持續讀取攝影機影像並做辨識
    ret, frame = cap.read()  # 從攝影機讀取一張畫面
    if not ret:  # 如果讀不到畫面
        print("讀不到攝影機畫面，程式結束。")  # 印出錯誤訊息
        break  # 跳出主迴圈

    h, w, _ = frame.shape  # 取得畫面的高與寬
    center_x = w // 2  # 計算畫面中心的 x 座標
    center_y = h // 2  # 計算畫面中心的 y 座標

    cv2.line(frame, (center_x, 0), (center_x, h), (255, 255, 255), 1)  # 在畫面上畫垂直中心線，形成十字準線的一部分
    cv2.line(frame, (0, center_y), (w, center_y), (255, 255, 255), 1)  # 在畫面上畫水平中心線，形成十字準線的一部分

    box_left = center_x - TARGET_ZONE_HALF_W  # 計算中心夾取框的左邊界
    box_right = center_x + TARGET_ZONE_HALF_W  # 計算中心夾取框的右邊界
    box_top = center_y - TARGET_ZONE_HALF_H  # 計算中心夾取框的上邊界
    box_bottom = center_y + TARGET_ZONE_HALF_H  # 計算中心夾取框的下邊界
    cv2.rectangle(frame, (box_left, box_top), (box_right, box_bottom), (0, 255, 255), 2)  # 在畫面中心畫出固定夾取區
    cv2.putText(frame, "FIXED PICK ZONE", (box_left - 10, box_top - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)  # 在夾取框上方標示這是固定夾取區

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)  # 對畫面做高斯模糊，降低雜訊與小反光造成的干擾
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)  # 把 BGR 畫面轉成 HSV 色彩空間，方便用顏色範圍做辨識

    mask_r = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)  # 建立紅色遮罩，因為紅色分兩段所以要相加
    mask_g = cv2.inRange(hsv, lower_green, upper_green)  # 建立綠色遮罩
    mask_b = cv2.inRange(hsv, lower_blue, upper_blue)  # 建立藍色遮罩

    target_r = find_target(mask_r, frame, "RED", (0, 0, 255))  # 尋找紅色目標並在畫面上標記
    target_g = find_target(mask_g, frame, "GREEN", (0, 255, 0))  # 尋找綠色目標並在畫面上標記
    target_b = find_target(mask_b, frame, "BLUE", (255, 0, 0))  # 尋找藍色目標並在畫面上標記
    target = choose_target([target_r, target_g, target_b], center_x, center_y)  # 從找到的所有顏色目標中選出主要目標

    status_msg = "SEARCH"  # 預設狀態文字為 SEARCH
    color_msg = "-"  # 預設顯示的顏色文字為 -
    zone_msg = "OUT"  # 預設顯示物體不在中心夾取區
    in_pick_zone = is_target_in_pick_zone(target, center_x, center_y)  # 先計算本幀目標是否在固定夾取區內

    if target is not None:  # 如果有找到目標
        color_msg = target["color"]  # 顯示目前辨識到的顏色
    zone_msg = "IN" if in_pick_zone else "OUT"  # 更新區域狀態顯示

    if state == "WAIT_PICK":  # 如果目前狀態是等待夾取完成
        remain = PICK_WAIT_SECONDS - (time.time() - pick_start_time)  # 計算還要等待幾秒
        if remain <= 0:  # 如果等待時間已經結束
            state = "WAIT_CLEAR"  # 切換到等待清空狀態，要求中心區必須先空掉
            clear_count = 0  # 清空區域計數從 0 開始重新累積
            stable_count = 0  # 清除穩定計數
            last_target = None  # 清除上一幀目標
        else:  # 如果還在等待時間內
            status_msg = f"WAIT_PICK {remain:.1f}s"  # 顯示還剩多少秒

    if state == "WAIT_CLEAR":  # 如果目前狀態是等待中心區物體被移除
        if in_pick_zone:  # 如果中心區內還有物體存在
            clear_count = 0  # 空白計數歸零，表示還不能重新允許下一次夾取
            status_msg = "WAIT_REMOVE_OBJECT"  # 提示必須先把目前物體移開
        else:  # 如果中心區已經沒有物體
            clear_count += 1  # 空白計數加一
            status_msg = f"WAIT_CLEAR {clear_count}/{CLEAR_FRAMES_REQUIRED}"  # 顯示還需要再空幾幀
            if clear_count >= CLEAR_FRAMES_REQUIRED:  # 如果已經連續空白足夠多幀
                state = "SEARCH"  # 重新回到 SEARCH 狀態，準備接受下一個新物體
                clear_count = 0  # 清空空白計數
                stable_count = 0  # 清空穩定計數
                last_target = None  # 清空上一幀目標
                status_msg = "READY_NEXT_OBJECT"  # 顯示已準備好接受下一個物體

    if state == "SEARCH":  # 如果目前狀態是 SEARCH
        if target is None:  # 如果目前沒有找到任何有效目標
            stable_count = 0  # 穩定計數歸零
            last_target = None  # 把上一幀目標清空
            status_msg = "SEARCH -> No target"  # 顯示沒有找到目標
        else:  # 如果有找到有效目標
            if in_pick_zone:  # 如果物體在固定夾取區內
                update_stable_count(target)  # 更新穩定計數
                status_msg = f"LOCKING {stable_count}/{LOCK_FRAMES}"  # 顯示正在鎖定第幾幀
                cv2.circle(frame, (target["cx"], target["cy"]), 6, (0, 255, 255), -1)  # 在物體中心畫一個實心點，表示已進入夾取區
            else:  # 如果物體不在固定夾取區內
                stable_count = 0  # 把穩定計數歸零
                last_target = None  # 把上一幀目標清空
                status_msg = "Place object on cross"  # 提示使用者把物體放回十字中心位置

            if in_pick_zone and stable_count >= LOCK_FRAMES and AUTO_RUN:  # 如果目標已穩定進入中心區，且自動模式已開啟
                cmd = f"PICK:{target['color']}"  # 依照辨識結果組合成 PICK 指令
                print(f"Detected color: {target['color']}")  # 先在終端機印出目前辨識到的顏色
                send_cmd(cmd)  # 把顏色夾取命令送給 ESP32
                pick_start_time = time.time()  # 記錄送出命令的時間
                state = "WAIT_PICK"  # 切換成等待夾取完成狀態
                stable_count = 0  # 清空穩定計數，避免重複觸發
                last_target = None  # 清空上一幀目標，等待下一次任務
                clear_count = 0  # 清空空白計數，下一階段將重新要求中心區變空
                status_msg = f"SEND {cmd}"  # 在畫面上顯示已送出哪個命令

    cv2.putText(frame, f"State: {status_msg}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2)  # 在畫面左上角顯示目前狀態
    cv2.putText(frame, f"Color: {color_msg}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2)  # 顯示目前辨識到的顏色
    cv2.putText(frame, f"Zone: {zone_msg}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (0, 255, 0) if zone_msg == "IN" else (0, 0, 255), 2)  # 顯示物體是否在固定夾取區內
    cv2.putText(frame, f"Stable: {stable_count}/{LOCK_FRAMES}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2)  # 顯示目前穩定幀數進度
    cv2.putText(frame, f"Clear: {clear_count}/{CLEAR_FRAMES_REQUIRED}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)  # 顯示中心區清空確認進度
    cv2.putText(frame, f"AUTO_RUN: {AUTO_RUN}", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 0), 2)  # 顯示自動模式是否啟用
    cv2.putText(frame, f"LAST CMD: {last_cmd}", (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)  # 顯示最後送給 ESP32 的命令

    cv2.imshow("Fixed Cross Auto Color Pick", frame)  # 顯示處理後的畫面視窗

    key = cv2.waitKey(1) & 0xFF  # 每一圈讀一次鍵盤，並只保留最後 8 位元的鍵值

    if key == ord("q"):  # 如果按下 q
        break  # 跳出主迴圈並結束程式
    elif key == ord("s"):  # 如果按下 s
        AUTO_RUN = not AUTO_RUN  # 切換自動模式開關
        stable_count = 0  # 切換模式時把穩定幀數清零
        last_target = None  # 切換模式時把上一幀目標清空
        clear_count = 0  # 切換模式時把清空幀數也清零
        if state != "WAIT_PICK":  # 如果當下不是手臂正在動作中
            state = "SEARCH"  # 讓狀態回到 SEARCH，方便重新開始
        print("AUTO_RUN =", AUTO_RUN)  # 在終端機印出切換後的狀態
    elif key == ord("h"):  # 如果按下 h
        send_cmd("HOME")  # 送出 HOME 命令，讓手臂回原位
    elif key == ord("o"):  # 如果按下 o
        send_cmd("OPEN")  # 送出 OPEN 命令，打開夾爪
    elif key == ord("c"):  # 如果按下 c
        send_cmd("CLOSE")  # 送出 CLOSE 命令，關閉夾爪
    elif key == ord("u"):  # 如果按下 u
        send_cmd("STATUS")  # 送出 STATUS 命令，要求 ESP32 回報目前狀態
    elif key == ord("r"):  # 如果按下 r
        send_cmd("PICK:RED")  # 手動測試紅色夾取
    elif key == ord("g"):  # 如果按下 g
        send_cmd("PICK:GREEN")  # 手動測試綠色夾取
    elif key == ord("b"):  # 如果按下 b
        send_cmd("PICK:BLUE")  # 手動測試藍色夾取

cap.release()  # 結束前釋放攝影機資源
cv2.destroyAllWindows()  # 結束前關閉所有 OpenCV 視窗
