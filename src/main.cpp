#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "variables.h"
#include "mpu.h"
#include "piloto_mode.h"
#include "manual_mode.h"
#include "motores.h"
#include <ESP32Servo.h>
#include <VL53L0X.h>

void setup()
{
  Serial.begin(115200);
  pinMode(pinLed, OUTPUT);
  Wire.begin();
  setup_manual_mode();
  setupMPU();
}

void loop()
{
  gyro_signals();
  loop_manual_mode();
  static int x = 0;
  if (x > 180000)
  {
    Serial.println("Motores apagados");
    apagarMotores();
  }
  else
  {
    x++;
  }
}
