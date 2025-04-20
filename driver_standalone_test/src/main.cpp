#include "boardsparams.h"
#include <SimpleFOC.h>
// BLDC driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(13, 16, 27, 12);
 
void setup() {
  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 50000;
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;
 
  // driver init
  driver.init();
 
  // enable driver
  driver.enable();
 
  _delay(1000);
  doParamsCheck();
}
 
void loop() {
    // setting pwm (A: 3V, B: 1V, C: 5V)
    driver.setPwm(3,1,5);
}