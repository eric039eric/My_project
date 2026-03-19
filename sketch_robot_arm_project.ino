#include <ESP32Servo.h>

Servo baseServo; // 底座馬達
Servo armServo;  // 大臂馬達

int basePin = 18; // 底座訊號線接 D18
int armPin = 19;  // 大臂訊號線接 D19

// 預設開機停在 90 度 (安全待機位置)
float currentBase = 90.0; 
float currentArm = 90.0;

void setup() {
  Serial.begin(115200);

  baseServo.attach(basePin);
  armServo.attach(armPin);

  // 開機先將兩軸鎖定在 90 度
  baseServo.write(currentBase);
  armServo.write(currentArm);

  Serial.println(" 雙軸系統準備完畢！");
  Serial.println("請輸入【目標底座角度】與【目標大臂角度】，中間用空格隔開");
  Serial.println("例如輸入150 45");
}

void loop() {
  if (Serial.available() > 0) {
    // 一次讀取兩個數字
    int targetBase = Serial.parseInt();
    int targetArm = Serial.parseInt();

    // 清除序列埠緩衝區的雜訊 (例如換行符號)
    while (Serial.available() > 0) {
      Serial.read();
    }

    // 安全機制：確認角度在合理範圍內
    if (targetBase >= 0 && targetBase <= 180 && targetArm >= 0 && targetArm <= 180) {
      Serial.print(" 準備平滑移動至 -> 底座: ");
      Serial.print(targetBase);
      Serial.print(" 度 | 大臂: ");
      Serial.print(targetArm);
      Serial.println(" 度");

      //  總工程師的多軸同步核心運算 
      // 算出誰要走的步數比較多，以此為基準
      int steps = max(abs(targetBase - currentBase), abs(targetArm - currentArm));

      if (steps > 0) {
        // 算出每走一步，底座和大臂分別要微調多少角度
        float baseStep = (targetBase - currentBase) / steps;
        float armStep = (targetArm - currentArm) / steps;

        // 開始同步微調移動
        for (int i = 1; i <= steps; i++) {
          baseServo.write(currentBase + baseStep * i);
          armServo.write(currentArm + armStep * i);
          delay(15); // 每微調一次停頓 15 毫秒 (數字越大動作越慢)
        }

        // 更新目前角度
        currentBase = targetBase;
        currentArm = targetArm;
      }
      Serial.println(" 動作完成！請輸入下一組指令：");

    } else if (targetBase != 0 || targetArm != 0) {
      Serial.println(" 警告：超出安全角度！請輸入 0-180。");
    }
  }
}