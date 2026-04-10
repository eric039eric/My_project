#include <WiFi.h>  // 匯入 Wi-Fi 函式庫，讓 ESP32 可以連上無線網路
#include <WiFiUdp.h>  // 匯入 UDP 函式庫，讓 ESP32 可以接收 Python 傳來的文字命令
#include <ESP32Servo.h>  // 匯入 ESP32 專用 Servo 函式庫，控制四顆伺服馬達

const char* ssid = "LAPTOP-ERIC039E 2916";  // 設定要連線的 Wi-Fi 名稱
const char* password = "zft2916eric";  // 設定要連線的 Wi-Fi 密碼

WiFiUDP Udp;  // 建立一個 UDP 物件，負責接收電腦傳來的指令
const int localPort = 8888;  // 設定 ESP32 監聽的 UDP Port
char packetBuffer[128];  // 建立一個字元陣列，用來暫存收到的 UDP 內容

Servo servos[4];  // 建立四顆 Servo 物件，分別對應 M1、M2、M3、M4
int pins[4] = {18, 19, 21, 22};  // 指定四顆馬達分別接到哪幾個 ESP32 腳位
int angles[4] = {90, 90, 90, 110};  // 紀錄四顆馬達目前角度的初始值

const int HOME_M1 = 90;  // 設定 HOME 姿態時 M1 的角度
const int HOME_M2 = 90;  // 設定 HOME 姿態時 M2 的角度
const int HOME_M3 = 90;  // 設定 HOME 姿態時 M3 的角度
const int HOME_M4 = 110;  // 設定 HOME 姿態時 M4 的角度

const int GRIP_OPEN = 115;  // 設定夾爪打開時的角度
const int GRIP_CLOSE = 70;  // 設定夾爪夾緊時的角度

const int PICK_DOWN_M2 = 34;  // 設定抓取時 M2 要下降到的固定角度
const int PICK_DOWN_M3 = 90;  // 設定抓取時 M3 要配合下降到的固定角度
const int PICK_UP_M2 = 85;  // 設定抓到物體後 M2 要抬回來的角度
const int PICK_UP_M3 = 95;  // 設定抓到物體後 M3 要抬回來的角度

const int PLACE_DOWN_M2 = 36;  // 設定放置物體時 M2 要下降到的固定角度
const int PLACE_DOWN_M3 = 70;  // 設定放置物體時 M3 要下降到的固定角度

const int PLACE_RED_M1 = 40;  // 設定紅色物體對應的放置區底座角度
const int PLACE_GREEN_M1 = 115;  // 設定綠色物體對應的放置區底座角度
const int PLACE_BLUE_M1 = 140;  // 設定藍色物體對應的放置區底座角度

const int DELAY_BIG = 15;  // 設定一般伺服大動作時每一步的延遲毫秒數
const int DELAY_GRIP = 10;  // 設定夾爪動作時每一步的延遲毫秒數

bool isBusy = false;  // 建立忙碌旗標，避免夾取流程中再次被新命令打斷

bool isNumericString(String s) {  // 定義函式：判斷某個字串是不是純數字
  s.trim();  // 去掉字串前後空白與換行
  if (s.length() == 0) return false;  // 如果字串長度是 0，就不是有效數字
  for (int i = 0; i < s.length(); i++) {  // 逐字檢查字串裡的每一個字元
    if (!isDigit(s[i])) return false;  // 如果出現任何不是數字的字元，直接回傳 false
  }
  return true;  // 如果所有字元都是數字，就回傳 true
}

void slowMove(int motorID, int target) {  // 定義函式：讓某顆馬達慢慢移動到指定角度
  if (motorID < 1 || motorID > 4) return;  // 如果馬達編號不在 1 到 4 之間，直接忽略
  int idx = motorID - 1;  // 將馬達編號轉成陣列索引值
  target = constrain(target, 0, 180);  // 把目標角度限制在 0 到 180 度範圍內

  if (abs(target - angles[idx]) <= 1) {  // 如果目標角度和目前角度差不到 1 度
    servos[idx].write(target);  // 直接寫入目標角度
    angles[idx] = target;  // 更新角度紀錄
    return;  // 結束函式
  }

  int d = (motorID == 4) ? DELAY_GRIP : DELAY_BIG;  // 如果是夾爪馬達 M4，使用夾爪專用速度，否則用一般速度

  if (angles[idx] < target) {  // 如果目前角度比目標角度小
    for (int p = angles[idx]; p <= target; p++) {  // 從目前角度一路增加到目標角度
      servos[idx].write(p);  // 每次寫入一個中間角度，讓動作變平滑
      delay(d);  // 每走一步延遲一下
    }
  } else {  // 如果目前角度比目標角度大
    for (int p = angles[idx]; p >= target; p--) {  // 從目前角度一路減少到目標角度
      servos[idx].write(p);  // 每次寫入一個中間角度，讓動作變平滑
      delay(d);  // 每走一步延遲一下
    }
  }

  angles[idx] = target;  // 動作結束後更新目前角度紀錄
}

void openGripper() {  // 定義函式：打開夾爪
  slowMove(4, GRIP_OPEN);  // 控制 M4 移到打開夾爪的角度
}

void closeGripper() {  // 定義函式：關閉夾爪
  slowMove(4, GRIP_CLOSE);  // 控制 M4 移到夾緊角度
}

void goHome() {  // 定義函式：讓整支手臂回到 HOME 姿態
  openGripper();  // 先打開夾爪，避免回原位時卡住物體
  slowMove(3, HOME_M3);  // 讓 M3 回到 HOME 角度
  slowMove(2, HOME_M2);  // 讓 M2 回到 HOME 角度
  slowMove(1, HOME_M1);  // 讓 M1 回到 HOME 角度
  slowMove(4, HOME_M4);  // 讓 M4 回到 HOME 角度
}

int getPlaceBaseByColor(String color) {  // 定義函式：根據顏色回傳對應的放置位置角度
  color.toUpperCase();  // 先把顏色字串轉成大寫，避免大小寫不一致造成判斷錯誤
  if (color == "RED") return PLACE_RED_M1;  // 如果是紅色，回傳紅色放置區角度
  if (color == "GREEN") return PLACE_GREEN_M1;  // 如果是綠色，回傳綠色放置區角度
  if (color == "BLUE") return PLACE_BLUE_M1;  // 如果是藍色，回傳藍色放置區角度
  return HOME_M1;  // 如果不是以上三種，保守回傳 HOME 的角度
}

void printStatus() {  // 定義函式：在序列埠印出目前系統狀態
  Serial.printf("STATUS | M1=%d M2=%d M3=%d M4=%d | busy=%d | WiFi=%s | IP=%s\n", angles[0], angles[1], angles[2], angles[3], isBusy, WiFi.status() == WL_CONNECTED ? "OK" : "NO", WiFi.localIP().toString().c_str());  // 用一行格式化輸出四顆馬達角度、忙碌狀態、Wi-Fi 狀態與 IP
}

void pickAndPlace(String color) {  // 定義函式：執行完整的固定點位抓取與顏色分類放置流程
  if (isBusy) {  // 如果目前手臂已經在忙
    Serial.println("PICK ignored: BUSY");  // 印出忽略訊息
    return;  // 直接離開函式，避免重複執行夾取
  }

  isBusy = true;  // 開始執行夾取前，先把忙碌旗標設成 true
  color.toUpperCase();  // 把輸入顏色轉成大寫，方便後續判斷

  Serial.print("PICK START -> ");  // 先印出抓取開始文字前半段
  Serial.println(color);  // 再印出這次要抓的顏色

  openGripper();  // 先打開夾爪，準備去夾物體
  slowMove(2, PICK_DOWN_M2);  // 讓 M2 下降到固定抓取高度
  slowMove(3, PICK_DOWN_M3);  // 讓 M3 下降到固定抓取姿態
  delay(150);  // 等待機構穩定

  closeGripper();  // 關閉夾爪，把物體夾住
  delay(250);  // 等待夾爪完全夾緊

  slowMove(3, PICK_UP_M3);  // 先讓 M3 抬回較高位置
  slowMove(2, PICK_UP_M2);  // 再讓 M2 抬回較高位置

  slowMove(1, getPlaceBaseByColor(color));  // 根據顏色把底座轉到對應的放置區
  slowMove(2, PLACE_DOWN_M2);  // 讓 M2 下降到放置高度
  slowMove(3, PLACE_DOWN_M3);  // 讓 M3 下降到放置姿態
  delay(150);  // 等待手臂穩定

  openGripper();  // 打開夾爪，把物體放下
  delay(250);  // 等待物體完全離開夾爪

  slowMove(3, PICK_UP_M3);  // 先讓 M3 抬回安全位置
  slowMove(2, PICK_UP_M2);  // 再讓 M2 抬回安全位置
  goHome();  // 最後讓整支手臂回到 HOME，等待下一次任務

  Serial.print("PICK DONE -> ");  // 印出完成訊息前半段
  Serial.println(color);  // 印出這次完成分類的顏色
  isBusy = false;  // 流程結束後把忙碌旗標恢復成 false
}

void handleCommand(String cmd, String source) {  // 定義函式：統一處理從序列埠或 UDP 收到的命令
  cmd.trim();  // 去掉命令字串前後的空白和換行
  if (cmd.length() == 0) return;  // 如果命令是空字串就直接忽略

  String ucmd = cmd;  // 先複製一份字串，避免直接改到原始輸入
  ucmd.toUpperCase();  // 把命令轉成大寫，讓判斷不受大小寫影響

  Serial.print(source);  // 印出命令來源，例如 SERIAL 或 UDP
  Serial.print(" RX -> ");  // 印出分隔箭頭
  Serial.println(ucmd);  // 印出實際收到的命令內容

  if (ucmd == "PING") {  // 如果收到 PING
    Serial.println("PONG");  // 回覆 PONG，表示裝置活著
    return;  // 結束函式
  }

  if (ucmd == "STATUS") {  // 如果收到 STATUS
    printStatus();  // 印出目前狀態
    return;  // 結束函式
  }

  if (ucmd == "HOME") {  // 如果收到 HOME
    goHome();  // 執行回 HOME 姿態
    printStatus();  // 印出目前狀態
    return;  // 結束函式
  }

  if (ucmd == "OPEN") {  // 如果收到 OPEN
    openGripper();  // 打開夾爪
    printStatus();  // 印出目前狀態
    return;  // 結束函式
  }

  if (ucmd == "CLOSE") {  // 如果收到 CLOSE
    closeGripper();  // 關閉夾爪
    printStatus();  // 印出目前狀態
    return;  // 結束函式
  }

  if (ucmd.startsWith("PICK:")) {  // 如果收到 PICK:RED、PICK:GREEN、PICK:BLUE 這類命令
    String color = ucmd.substring(5);  // 把冒號後面的顏色字串取出來
    pickAndPlace(color);  // 執行固定點位抓取與分類放置流程
    printStatus();  // 印出目前狀態
    return;  // 結束函式
  }

  if (isBusy) {  // 如果目前正在執行夾取流程
    if (ucmd.startsWith("BASE:") || ucmd.startsWith("M") || isNumericString(ucmd)) {  // 若又收到單馬達或底座角度命令
      Serial.println("IGNORED: BUSY");  // 印出正在忙碌中所以忽略此命令
      return;  // 直接離開函式
    }
  }

  if (isNumericString(ucmd)) {  // 如果收到的是純數字角度
    int angle = ucmd.toInt();  // 把字串轉成整數
    slowMove(1, angle);  // 預設把這個數字當成 M1 的角度
    printStatus();  // 印出目前狀態
    return;  // 結束函式
  }

  if (ucmd.startsWith("BASE:")) {  // 如果收到 BASE:角度 命令
    int angle = ucmd.substring(5).toInt();  // 把冒號後面的角度字串轉成整數
    slowMove(1, angle);  // 控制 M1 移到該角度
    printStatus();  // 印出目前狀態
    return;  // 結束函式
  }

  if (ucmd.startsWith("M")) {  // 如果收到 M1:90、M2:45 這類單顆馬達測試命令
    int colon = ucmd.indexOf(':');  // 找到冒號的位置
    if (colon > 1) {  // 如果格式正確且有找到冒號
      int motorID = ucmd.substring(1, colon).toInt();  // 取出 M 後面到冒號前的馬達編號
      int angle = ucmd.substring(colon + 1).toInt();  // 取出冒號後面的角度值
      if (motorID >= 1 && motorID <= 4) {  // 如果馬達編號在合法範圍內
        slowMove(motorID, angle);  // 控制對應馬達移動到指定角度
        printStatus();  // 印出目前狀態
        return;  // 結束函式
      }
    }
  }

  Serial.println("Unknown command");  // 如果以上都不符合，就回報這是未知命令
}

void setup() {  // setup 函式只會在 ESP32 開機時執行一次
  Serial.begin(115200);  // 啟用序列埠通訊，鮑率設為 115200
  delay(1000);  // 等待 1 秒，讓系統穩定啟動

  Serial.println("=== ESP32 Fixed Cross Single-Cycle Auto Pick Controller ===");  // 印出目前控制器版本名稱

  for (int i = 0; i < 4; i++) {  // 用迴圈初始化四顆伺服馬達
    servos[i].setPeriodHertz(50);  // 設定伺服輸出週期為 50Hz
    servos[i].attach(pins[i], 500, 2400);  // 把伺服附加到指定腳位並設定脈波寬度範圍
    servos[i].write(angles[i]);  // 讓每顆馬達先轉到初始角度
    Serial.printf("Servo %d attached on D%d, angle=%d\n", i + 1, pins[i], angles[i]);  // 印出每顆伺服的初始化資訊
    delay(400);  // 每顆馬達初始化後稍微停一下，降低啟動瞬間負載
  }

  Serial.println("Connecting to WiFi...");  // 印出開始連線 Wi-Fi 的提示文字
  WiFi.begin(ssid, password);  // 使用設定好的 SSID 與密碼連線到 Wi-Fi

  while (WiFi.status() != WL_CONNECTED) {  // 只要尚未成功連上 Wi-Fi 就持續等待
    delay(500);  // 每次等待半秒
    Serial.print(".");  // 在序列埠印出點點提示連線中
  }

  Serial.println("\nWiFi Connected!");  // 成功連上 Wi-Fi 後印出成功訊息
  Serial.print("My IP: ");  // 印出 IP 提示字樣
  Serial.println(WiFi.localIP());  // 印出 ESP32 目前的 IP 位址

  Udp.begin(localPort);  // 開始監聽指定的 UDP Port
  Serial.printf("UDP listening on %d\n", localPort);  // 印出目前監聽的 Port

  goHome();  // 啟動完成後先讓手臂回到 HOME 姿態
  printStatus();  // 印出啟動後的初始狀態
}

void loop() {  // loop 函式會一直重複執行，是 ESP32 的主循環
  if (Serial.available() > 0) {  // 如果序列埠有收到資料
    String serialCmd = Serial.readStringUntil('\n');  // 讀取一整行序列埠命令直到換行為止
    handleCommand(serialCmd, "SERIAL");  // 把這條命令交給 handleCommand 處理，並標記來源是 SERIAL
  }

  int packetSize = Udp.parsePacket();  // 檢查是否收到新的 UDP 封包，並取得封包大小
  if (packetSize > 0) {  // 如果有收到封包
    int len = Udp.read(packetBuffer, sizeof(packetBuffer) - 1);  // 把封包內容讀進字元陣列，並保留最後一格給結束符號
    if (len > 0) {  // 如果真的有讀到內容
      packetBuffer[len] = '\0';  // 在字串最後補上結束符號，讓它成為合法 C 字串
      String udpCmd = String(packetBuffer);  // 把字元陣列轉成 Arduino String
      handleCommand(udpCmd, "UDP");  // 把這條 UDP 命令交給 handleCommand 處理，並標記來源是 UDP
    }
  }
}
