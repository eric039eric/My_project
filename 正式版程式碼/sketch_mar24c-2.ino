#include <WiFi.h>        // 匯入 Wi-Fi 函式庫，讓 ESP32 可以連上無線網路
#include <WiFiUdp.h>     // 匯入 UDP 函式庫，讓 ESP32 可以接收 Python 傳來的文字命令
#include <ESP32Servo.h>  // 匯入 ESP32 專用 Servo 函式庫，控制四顆伺服馬達

const char* ssid = "LAPTOP-ERIC039E 2916";  // 設定要連線的 Wi-Fi 名稱
const char* password = "zft2916eric";       // 設定要連線的 Wi-Fi 密碼

// ===== 固定 IP 設定 =====
// 若你的筆電熱點/路由器是 192.168.137.1，下面這組通常可直接使用。
// 若你的網路不是 192.168.137.x，請把這三個值改成同一網段。
IPAddress local_IP(192, 168, 137, 50);      // ESP32 固定 IP
IPAddress gateway(192, 168, 137, 1);        // 路由器 / Windows 熱點閘道
IPAddress subnet(255, 255, 255, 0);         // 子網路遮罩
IPAddress primaryDNS(8, 8, 8, 8);           // 主要 DNS
IPAddress secondaryDNS(1, 1, 1, 1);         // 次要 DNS

WiFiUDP Udp;                  // 建立一個 UDP 物件，負責接收電腦傳來的指令
const int localPort = 8888;   // 設定 ESP32 監聽的 UDP Port
char packetBuffer[128];       // 建立一個字元陣列，用來暫存收到的 UDP 內容

Servo servos[4];                  // 建立四顆 Servo 物件，分別對應 M1、M2、M3、M4
int pins[4] = {18, 19, 21, 22};   // 指定四顆馬達分別接到哪幾個 ESP32 腳位
int angles[4] = {90, 90, 90, 110}; // 紀錄四顆馬達目前角度的初始值

const int HOME_M1 = 90;   // 設定 HOME 姿態時 M1 的角度
const int HOME_M2 = 90;   // 設定 HOME 姿態時 M2 的角度
const int HOME_M3 = 90;   // 設定 HOME 姿態時 M3 的角度
const int HOME_M4 = 110;  // 設定 HOME 姿態時 M4 的角度

const int GRIP_OPEN = 115;   // 設定夾爪打開時的角度
const int GRIP_CLOSE = 70;   // 設定夾爪夾緊時的角度

const int PICK_DOWN_M2 = 34;  // 設定抓取時 M2 要下降到的固定角度
const int PICK_DOWN_M3 = 90;  // 設定抓取時 M3 要配合下降到的固定角度
const int PICK_UP_M2 = 85;    // 設定抓到物體後 M2 要抬回來的角度
const int PICK_UP_M3 = 95;    // 設定抓到物體後 M3 要抬回來的角度

const int PLACE_DOWN_M2 = 36;  // 設定放置物體時 M2 要下降到的固定角度
const int PLACE_DOWN_M3 = 70;  // 設定放置物體時 M3 要下降到的固定角度

const int PLACE_RED_M1 = 40;    // 設定紅色物體對應的放置區底座角度
const int PLACE_GREEN_M1 = 115; // 設定綠色物體對應的放置區底座角度
const int PLACE_BLUE_M1 = 140;  // 設定藍色物體對應的放置區底座角度

const int DELAY_BIG = 15;   // 設定一般伺服大動作時每一步的延遲毫秒數
const int DELAY_GRIP = 10;  // 設定夾爪動作時每一步的延遲毫秒數

bool isBusy = false;  // 建立忙碌旗標，避免夾取流程中再次被新命令打斷

bool isNumericString(String s) {  // 定義函式：判斷某個字串是不是純數字
  s.trim();
  if (s.length() == 0) return false;
  for (int i = 0; i < s.length(); i++) {
    if (!isDigit(s[i])) return false;
  }
  return true;
}

void slowMove(int motorID, int target) {  // 定義函式：讓某顆馬達慢慢移動到指定角度
  if (motorID < 1 || motorID > 4) return;
  int idx = motorID - 1;
  target = constrain(target, 0, 180);

  if (abs(target - angles[idx]) <= 1) {
    servos[idx].write(target);
    angles[idx] = target;
    return;
  }

  int d = (motorID == 4) ? DELAY_GRIP : DELAY_BIG;

  if (angles[idx] < target) {
    for (int p = angles[idx]; p <= target; p++) {
      servos[idx].write(p);
      delay(d);
    }
  } else {
    for (int p = angles[idx]; p >= target; p--) {
      servos[idx].write(p);
      delay(d);
    }
  }

  angles[idx] = target;
}

void openGripper() {
  slowMove(4, GRIP_OPEN);
}

void closeGripper() {
  slowMove(4, GRIP_CLOSE);
}

void goHome() {
  openGripper();
  slowMove(3, HOME_M3);
  slowMove(2, HOME_M2);
  slowMove(1, HOME_M1);
  slowMove(4, HOME_M4);
}

int getPlaceBaseByColor(String color) {
  color.toUpperCase();
  if (color == "RED") return PLACE_RED_M1;
  if (color == "GREEN") return PLACE_GREEN_M1;
  if (color == "BLUE") return PLACE_BLUE_M1;
  return HOME_M1;
}

void printStatus() {
  Serial.printf(
    "STATUS | M1=%d M2=%d M3=%d M4=%d | busy=%d | WiFi=%s | IP=%s | GW=%s\n",
    angles[0], angles[1], angles[2], angles[3], isBusy,
    WiFi.status() == WL_CONNECTED ? "OK" : "NO",
    WiFi.localIP().toString().c_str(),
    WiFi.gatewayIP().toString().c_str()
  );
}

void connectWiFiStatic() {
  Serial.println("Connecting to WiFi with static IP...");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(300);

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Failed to configure static IP");
  }

  WiFi.begin(ssid, password);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    retry++;

    if (retry >= 60) {
      Serial.println("\nWiFi connect timeout, retrying...");
      WiFi.disconnect();
      delay(500);
      WiFi.begin(ssid, password);
      retry = 0;
    }
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Static IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Gateway : ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("Subnet  : ");
  Serial.println(WiFi.subnetMask());
  Serial.print("DNS 1   : ");
  Serial.println(WiFi.dnsIP(0));
  Serial.print("DNS 2   : ");
  Serial.println(WiFi.dnsIP(1));
}

void pickAndPlace(String color) {
  if (isBusy) {
    Serial.println("PICK ignored: BUSY");
    return;
  }

  isBusy = true;
  color.toUpperCase();

  Serial.print("PICK START -> ");
  Serial.println(color);

  openGripper();
  slowMove(2, PICK_DOWN_M2);
  slowMove(3, PICK_DOWN_M3);
  delay(150);

  closeGripper();
  delay(250);

  slowMove(3, PICK_UP_M3);
  slowMove(2, PICK_UP_M2);

  slowMove(1, getPlaceBaseByColor(color));
  slowMove(2, PLACE_DOWN_M2);
  slowMove(3, PLACE_DOWN_M3);
  delay(150);

  openGripper();
  delay(250);

  slowMove(3, PICK_UP_M3);
  slowMove(2, PICK_UP_M2);
  goHome();

  Serial.print("PICK DONE -> ");
  Serial.println(color);
  isBusy = false;
}

void handleCommand(String cmd, String source) {
  cmd.trim();
  if (cmd.length() == 0) return;

  String ucmd = cmd;
  ucmd.toUpperCase();

  Serial.print(source);
  Serial.print(" RX -> ");
  Serial.println(ucmd);

  if (ucmd == "PING") {
    Serial.println("PONG");
    return;
  }

  if (ucmd == "STATUS") {
    printStatus();
    return;
  }

  if (ucmd == "HOME") {
    goHome();
    printStatus();
    return;
  }

  if (ucmd == "OPEN") {
    openGripper();
    printStatus();
    return;
  }

  if (ucmd == "CLOSE") {
    closeGripper();
    printStatus();
    return;
  }

  if (ucmd == "PICK:RED" || ucmd == "PICK:GREEN" || ucmd == "PICK:BLUE") {
    String color = ucmd.substring(5);
    pickAndPlace(color);
    printStatus();
    return;
  }

  if (isBusy) {
    if (ucmd.startsWith("BASE:") || ucmd.startsWith("M") || isNumericString(ucmd)) {
      Serial.println("IGNORED: BUSY");
      return;
    }
  }

  if (isNumericString(ucmd)) {
    int angle = ucmd.toInt();
    slowMove(1, angle);
    printStatus();
    return;
  }

  if (ucmd.startsWith("BASE:")) {
    int angle = ucmd.substring(5).toInt();
    slowMove(1, angle);
    printStatus();
    return;
  }

  if (ucmd.startsWith("M")) {
    int colon = ucmd.indexOf(':');
    if (colon > 1) {
      int motorID = ucmd.substring(1, colon).toInt();
      int angle = ucmd.substring(colon + 1).toInt();
      if (motorID >= 1 && motorID <= 4) {
        slowMove(motorID, angle);
        printStatus();
        return;
      }
    }
  }

  Serial.println("Unknown command");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== ESP32 Fixed Cross Single-Cycle Auto Pick Controller (Static IP) ===");

  for (int i = 0; i < 4; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(pins[i], 500, 2400);
    servos[i].write(angles[i]);
    Serial.printf("Servo %d attached on D%d, angle=%d\n", i + 1, pins[i], angles[i]);
    delay(400);
  }

  connectWiFiStatic();

  Udp.begin(localPort);
  Serial.printf("UDP listening on %d\n", localPort);

  goHome();
  printStatus();
}

void loop() {
  if (Serial.available() > 0) {
    String serialCmd = Serial.readStringUntil('\n');
    handleCommand(serialCmd, "SERIAL");
  }

  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    int len = Udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';
      String udpCmd = String(packetBuffer);
      handleCommand(udpCmd, "UDP");
    }
  }
}
