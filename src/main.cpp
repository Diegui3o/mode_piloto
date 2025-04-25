#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "variables.h"
#include "mpu.h"
#include "piloto_mode.h"
#include <ESP32Servo.h>
#include <VL53L0X.h>

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  setupMPU();
}

void loop()
{
  loopMPU();
  gyro_signals();
  Serial.println(x_roll[0]);
  Serial.println(x_pitch[0]);
}
