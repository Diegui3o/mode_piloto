#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "variables.h"
#include "mpu.h"
#include "piloto_mode.h"
#include <ESP32Servo.h>
#include <VL53L0X.h>
#include "motores.h"

void setup()
{
  Serial.begin(115200);
  pinMode(pinLed, OUTPUT);
  Wire.begin();
  setupMPU();
  delay(1000);
  setup_pilote_mode();
}

void loop()
{
  gyro_signals();
  loop_pilote_mode();
  static int i = 0;
  if (i < 1000)
  {
    digitalWrite(pinLed, HIGH);
    apagarMotores();
  }
  else
  {
    i++;
  }
}
