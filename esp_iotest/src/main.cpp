#include <Arduino.h>
#include <SimpleFOC.h>

uint8_t Ledpin = 5;
float target_pin = 5;
uint8_t Lastpin = 255; // 初始化为不存在的pin，保证第一次触发pinMode
Commander command = Commander(Serial);

unsigned long previousMillis = 0;
const long interval = 500; // LED闪烁间隔
bool ledState = false;     // 当前LED状态

void doTarget(char* cmd) { 
  command.scalar(&target_pin, cmd); 
  uint8_t newPin = uint8_t(target_pin);
  if(newPin < 64) { 
    Ledpin = newPin;
    Serial.printf("Received LED pin: %d\n", Ledpin);
  }
}

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
  command.add('T', doTarget, "target pin number");

  pinMode(Ledpin, OUTPUT);
  digitalWrite(Ledpin, LOW);
  Serial.println(F("Board ready."));
}

void loop() {
  unsigned long currentMillis = millis();

  // 非阻塞方式控制LED闪烁
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // 如果切换了pin，关闭之前pin
    if (Lastpin != Ledpin) {
      if(Lastpin != 255) digitalWrite(Lastpin, LOW); //关闭之前的pin
      pinMode(Ledpin, OUTPUT);
      Lastpin = Ledpin;
      ledState = false;  // 重置状态确保从OFF开始
      Serial.printf("Changed LED pin: %d\n", Ledpin);
    }

    ledState = !ledState;                // 切换LED状态
    digitalWrite(Ledpin, ledState);      // 更新LED状态
  }
  delay(50);
  // 必须放到loop()的主逻辑中，以保证实时处理串口指令
  command.run();
}
