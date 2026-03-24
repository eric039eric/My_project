#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// ================= Wi-Fi 設定 =================
const char* ssid = "LAPTOP-ERIC039E 2916";
const char* password = "zft2916eric";

// ================= UDP 設定 =================
WiFiUDP Udp;
const int localPort = 8888;
char packetBuffer[128];

// ================= 伺服設定 =================
Servo servos[4];
int pins[4] = {18, 19, 21, 22};      // M1, M2, M3, M4
int angles[4] = {90, 90, 90, 110};   // 目前角度

// ================= 基本姿態 =================
// 這些角度一定要依你的機械臂實際校正
const int HOME_M1 = 90;
const int HOME_M2 = 90;
const int HOME_M3 = 90;
const int HOME_M4 = 110;   // 夾爪張開

const int GRIP_OPEN = 110;
const int GRIP_CLOSE = 70;

// 抓取姿態
const int PICK_DOWN_M2 = 110;
const int PICK_DOWN_M3 = 125;
const int PICK_UP_M2 = 85;
const int PICK_UP_M3 = 95;

// 放置姿態
const int PLACE_DOWN_M2 = 105;
const int PLACE_DOWN_M3 = 115;

// 顏色放置區底座角度
const int PLACE_RED_M1 = 40;
const int PLACE_GREEN_M1 = 90;
const int PLACE_BLUE_M1 = 140;

// 速度
const int DELAY_BIG = 15;
const int DELAY_GRIP = 10;

bool isBusy = false;

// --------------------------------------------------
bool isNumericString(String s) {
  s.trim();
  if (s.length() == 0) return false;
  for (int i = 0; i < s.length(); i++) {
    if (!isDigit(s[i])) return false;
  }
  return true;
}

// --------------------------------------------------
void slowMove(int motorID, int target) {
  if (motorID < 1 || motorID > 4) return;

  int idx = motorID - 1;
  target = constrain(target, 0, 180);

  if (abs(target - angles[idx]) <= 1) return;

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

// --------------------------------------------------
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

// --------------------------------------------------
int getPlaceBaseByColor(String color) {
  color.toUpperCase();
  if (color == "RED") return PLACE_RED_M1;
  if (color == "GREEN") return PLACE_GREEN_M1;
  if (color == "BLUE") return PLACE_BLUE_M1;
  return HOME_M1;
}

// --------------------------------------------------
void pickAndPlace(String color) {
  isBusy = true;
  color.toUpperCase();

  Serial.print("PICK START -> ");
  Serial.println(color);

  // 目前底座 M1 應該已被 Python 對準目標
  openGripper();

  // 下去夾取
  slowMove(2, PICK_DOWN_M2);
  slowMove(3, PICK_DOWN_M3);
  delay(150);

  // 夾住
  closeGripper();
  delay(250);

  // 抬起
  slowMove(3, PICK_UP_M3);
  slowMove(2, PICK_UP_M2);

  // 轉到放置區
  slowMove(1, getPlaceBaseByColor(color));

  // 放下
  slowMove(2, PLACE_DOWN_M2);
  slowMove(3, PLACE_DOWN_M3);
  delay(150);

  // 放開
  openGripper();
  delay(250);

  // 抬起回家
  slowMove(3, PICK_UP_M3);
  slowMove(2, PICK_UP_M2);
  goHome();

  Serial.print("PICK DONE -> ");
  Serial.println(color);

  isBusy = false;
}

// --------------------------------------------------
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  String ucmd = cmd;
  ucmd.toUpperCase();

  Serial.print("CMD = ");
  Serial.println(ucmd);

  // 舊版相容：純數字 -> 控制底座
  if (isNumericString(ucmd)) {
    int angle = ucmd.toInt();
    slowMove(1, angle);
    return;
  }

  // 基本命令
  if (ucmd == "HOME") {
    goHome();
    return;
  }

  if (ucmd == "OPEN") {
    openGripper();
    return;
  }

  if (ucmd == "CLOSE") {
    closeGripper();
    return;
  }

  // BASE:120
  if (ucmd.startsWith("BASE:")) {
    int angle = ucmd.substring(5).toInt();
    slowMove(1, angle);
    return;
  }

  // M1:120 / M2:90 / M3:130 / M4:70
  if (ucmd.startsWith("M")) {
    int colon = ucmd.indexOf(':');
    if (colon > 1) {
      int motorID = ucmd.substring(1, colon).toInt();
      int angle = ucmd.substring(colon + 1).toInt();
      if (motorID >= 1 && motorID <= 4) {
        slowMove(motorID, angle);
      }
    }
    return;
  }

  // PICK:RED / PICK:GREEN / PICK:BLUE
  if (ucmd.startsWith("PICK:")) {
    String color = ucmd.substring(5);
    pickAndPlace(color);
    return;
  }

  Serial.println("Unknown command");
}

// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== ESP32 Auto Pick Controller ===");

  // 依序上電，避免瞬間暴衝
  for (int i = 0; i < 4; i++) {
    servos[i].attach(pins[i], 500, 2400);
    servos[i].write(angles[i]);
    Serial.printf("Servo %d attached on D%d, angle=%d\n", i + 1, pins[i], angles[i]);
    delay(400);
  }

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("My IP: ");
  Serial.println(WiFi.localIP());

  Udp.begin(localPort);
  Serial.printf("UDP listening on %d\n", localPort);

  goHome();
}

// --------------------------------------------------
void loop() {
  int packetSize = Udp.parsePacket();
  if (!packetSize) return;

  int len = Udp.read(packetBuffer, sizeof(packetBuffer) - 1);
  if (len <= 0) return;
  packetBuffer[len] = '\0';

  String cmd = String(packetBuffer);

  if (isBusy) {
    Serial.println("BUSY -> command ignored");
    return;
  }

  handleCommand(cmd);
}
