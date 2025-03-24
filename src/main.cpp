#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "variables.h"
#include "mpu.h"
#include "pilote_mode.h"
#include <ESP32Servo.h>
#include <VL53L0X.h>

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  setupMPU();
  setup_pilote_mode();
}

void loop()
{
  gyro_signals();
  loop_pilote_mode();
}
