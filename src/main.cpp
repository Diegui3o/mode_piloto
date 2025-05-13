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
  digitalWrite(pinLed, HIGH);
  setupMPU();
  channelInterrupHandler();
  setupMotores();
  setup_manual_mode();
  LoopTimer = micros();
  digitalWrite(pinLed, LOW);
}

void loop()
{
  gyro_signals();
  loop_yaw();
  static uint32_t last_time = 0;
  float dt = (micros() - last_time) / 1e6;
  if (dt < 0.002)
    return; // Esperar a 2ms (500Hz)

  // 2. Ejecutar LQR
  loop_manual_mode(dt);

  last_time = micros();
}
