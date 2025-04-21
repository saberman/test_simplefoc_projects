// MKS DUAL FOC ABI编码器测试例程 测试硬件：MKS DUAL FOC V3.1
// 测试用编码器 AS5047P,CPR=4000


#include <SimpleFOC.h>
#include <SPI.h>
// MagneticSensorI2C(uint8_t _chip_address, float _cpr, uint8_t _angle_register_msb)
//  chip_address  I2C chip address
//  bit_resolution  resolution of the sensor
//  angle_register_msb  angle read register msb
//  bits_used_msb  number of used bits in msb register
// 
// make sure to read the chip address and the chip angle register msb value from the datasheet
// also in most cases you will need external pull-ups on SDA and SCL lines!!!!!
//
// For AS5058B
// MagneticSensorI2C sensor = MagneticSensorI2C(0x40, 14, 0xFE, 8);

// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
// cs              - SPI芯片选择引脚
// bit_resolution  - 传感器分辨率
// angle_register  - （可选的）角度读取寄存器 - 默认 0x3FFF
//MagneticSensorSPI as5047u = MagneticSensorSPI(10, 14, 0x3FFF);
// 快速配置
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 5);

void setup() {
  // 监视点
  Serial.begin(115200);

  // 初始化磁性传感器硬件
  //SPIClass _spi = SPI;

  sensor.init();

  Serial.println("as5047P ready");
  _delay(1000);
}

/**
 * @brief Main loop function that continuously reads and displays angle and velocity data
 * 
 * This function:
 * - Updates the AS5047P sensor readings
 * - Prints the current angle to Serial
 * - Prints the current velocity to Serial
 * - Data is tab-separated with angle and velocity on each line
 */
void loop() {
  // 在终端显示角度和角速度
  sensor.update();
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}

//针对ABI接口（0）号
//Encoder encoder = Encoder(19,18,1000,15); //A0;B0;编码器PPR,PPR=CPR/4;I0

//针对ABI接口（1）号
Encoder encoder = Encoder(23,5,1000,13); //A1;B1;编码器PPR,PPR=CPR/4;I1

void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

void setup1() {
  Serial.begin(115200);
  encoder.quadrature = Quadrature::ON;
  encoder.pullup = Pullup::USE_EXTERN;

  encoder.init();
  // 硬件中断使能
  encoder.enableInterrupts(doA, doB);

  Serial.println("Encoder ready");
  _delay(1000);
}
void loop1() {
  // 输出角度和角速度
  Serial.print(encoder.getAngle());
  Serial.print("\t");
  Serial.println(encoder.getVelocity());
}
