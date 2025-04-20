#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7); // 设置电机极对数
BLDCDriver3PWM driver = BLDCDriver3PWM(13, 16, 27, 12); // 设置驱动器引脚
Commander command = Commander(Serial);

void onMotor(char* cmd) { command.motor(&motor, cmd); } // 定义电机命令处理函数

void setup() {
  Serial.begin(115200);
  motor.linkDriver(&driver);
  motor.init();
  motor.useMonitoring(Serial); // 启用监控功能
  command.add('M', onMotor, "my motor"); // 添加电机命令
  motor.monitor_downsample = 0; // disable monitor at first - optional
}

void loop() {
  motor.loopFOC();
  motor.move();
  motor.monitor(); // 输出监控信息
  command.run(); // 处理串口命令
}
