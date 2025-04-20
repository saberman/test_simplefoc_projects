/**
 *
 * Velocity motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target velocity (in radians per second) from serial terminal
 *
 *
 * By using the serial terminal set the velocity value you want to motor to obtain
 *
 */
#include <SimpleFOC.h>

// magnetic sensor instance - SPI
//MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);
// magnetic sensor instance - MagneticSensorI2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
//MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(13, 16, 27, 12);
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// velocity set point variable
float target_velocity = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);
#if defined(ESP_H)
  Serial.println(F("ESP_H:"));
  _delay(300); 
#endif  
#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32)
  Serial.println(F("ARDUINO_ARCH_ESP32:"));
  _delay(300); 
#endif

#if defined(ESP_H) && defined(SOC_MCPWM_SUPPORTED)
  Serial.println(F("SOC_MCPWM_SUPPORTED:"));
  _delay(300); 
#endif

  Serial.println(F("Set 1:"));
  _delay(300);
  // initialise magnetic sensor hardware
  sensor.init();
  Serial.println(F("Set 2:"));
  _delay(300);
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  Serial.println(F("Set 3:"));
  _delay(300);
  
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();

  Serial.println(F("Set 4:"));
  _delay(300);
  // link the motor and the driver
  motor.linkDriver(&driver);

  Serial.println(F("Set 5:"));
  _delay(300);
  
  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.1f;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  motor.voltage_limit = 1;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  // comment out if not needed
  motor.useMonitoring(Serial);

  Serial.println(F("Set 6:"));
  _delay(300);

  // initialize motor
  motor.init();

  Serial.println(F("Set 7:"));
  _delay(300);

  // align sensor and start FOC
  motor.initFOC();

  Serial.println(F("Set 8:"));
  _delay(300);

  // add target command T
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  command.run();
}