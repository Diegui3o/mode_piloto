#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include <VL53L0X.h>
#include "variables.h"
#include "mpu.h"

#define SDA_MPU 21
#define SCL_MPU 22
#define SDA_TOF 4
#define SCL_TOF 5

// Declare and initialize Q and R
float Q[2][2] = {{0.001, 0}, {0, 0.003}}; // Process noise covariance matrix
float R = 0.03;                           // Measurement noise covariance

// Función para el filtro de Kalman (roll)
void kalmanUpdateRoll(float accAngleRoll, float RateRoll, float dt)
{
  // State transition matrix
  float F[2][2] = {{1, -dt}, {0, 1}};
  float G[2] = {dt, 0};
  float H[2] = {1, 0};

  // State prediction
  float x_pred[2];
  x_pred[0] = F[0][0] * x_roll[0] + F[0][1] * x_roll[1] + G[0] * RateRoll;
  x_pred[1] = F[1][0] * x_roll[0] + F[1][1] * x_roll[1] + G[1] * RateRoll;

  // Error covariance prediction
  float P_pred[2][2];
  P_pred[0][0] = F[0][0] * P_roll[0][0] + F[0][1] * P_roll[1][0];
  P_pred[0][1] = F[0][0] * P_roll[0][1] + F[0][1] * P_roll[1][1];
  P_pred[1][0] = F[1][0] * P_roll[0][0] + F[1][1] * P_roll[1][0];
  P_pred[1][1] = F[1][0] * P_roll[0][1] + F[1][1] * P_roll[1][1];

  // Add process noise
  P_pred[0][0] += Q[0][0];
  P_pred[1][1] += Q[1][1];

  // Kalman gain
  float S = H[0] * P_pred[0][0] * H[0] + R;
  float K[2];
  K[0] = P_pred[0][0] * H[0] / S;
  K[1] = P_pred[1][0] * H[0] / S;

  // Measurement residual
  float d_roll = accAngleRoll - (H[0] * x_pred[0]);

  // State update
  x_roll[0] = x_pred[0] + K[0] * d_roll;
  x_roll[1] = x_pred[1] + K[1] * d_roll;

  // Error covariance update
  P_roll[0][0] = (1 - K[0] * H[0]) * P_pred[0][0];
  P_roll[0][1] = (1 - K[0] * H[0]) * P_pred[0][1];
  P_roll[1][0] = -K[1] * H[0] * P_pred[0][0] + P_pred[1][0];
  P_roll[1][1] = -K[1] * H[0] * P_pred[0][1] + P_pred[1][1];

  // Forgetting factor adaptation
  C = (C * 0.99) + (0.01 * d_roll * d_roll);
  lambda = 1.0; // Simplified for now, implement your rule here

  // Update error covariance with forgetting factor
  P_roll[0][0] = (C + lambda * d_roll * d_roll) / C * P_roll[0][0];
  P_roll[1][1] = (C + lambda * d_roll * d_roll) / C * P_roll[1][1];
}

// Función para el filtro de Kalman (pitch)
void kalmanUpdatePitch(float accAnglePitch, float gyroRatePitch)
{
  // Predicción del estado
  x_pitch[0] += dt * (gyroRatePitch - x_pitch[1]); // Actualiza el ángulo
  x_pitch[1] = x_pitch[1];                         // El bias del giroscopio no cambia

  // Predicción de la covarianza del error
  P_pitch[0][0] += dt * (P_pitch[1][1] * dt - P_pitch[0][1] - P_pitch[1][0] + Q_angle);
  P_pitch[0][1] -= dt * P_pitch[1][1];
  P_pitch[1][0] -= dt * P_pitch[1][1];
  P_pitch[1][1] += Q_gyro * dt;

  // Ganancia de Kalman
  float S = P_pitch[0][0] + R_angle;
  float K[2];
  K[0] = P_pitch[0][0] / S;
  K[1] = P_pitch[1][0] / S;

  float y = accAnglePitch - x_pitch[0];
  x_pitch[0] += K[0] * y;
  x_pitch[1] += K[1] * y;

  P_pitch[0][0] -= K[0] * P_pitch[0][0];
  P_pitch[0][1] -= K[0] * P_pitch[0][1];
  P_pitch[1][0] -= K[1] * P_pitch[0][0];
  P_pitch[1][1] -= K[1] * P_pitch[0][1];
}

void gyro_signals(void)
{
  // Leer los valores de los sensores
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccX = Wire.read() << 8 | Wire.read();
  int16_t AccY = Wire.read() << 8 | Wire.read();
  int16_t AccZ = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  GyroXdps = GyroX / 131.0;
  GyroYdps = GyroY / 131.0;
  GyroZdps = GyroZ / 131.0;

  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58;

  if (AccZ != 0)
  {
    accAngleRoll = atan2(AccY, AccZ) * 57.29;                              // Ángulo de roll (grados)
    accAnglePitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 57.29; // Ángulo de pitch (grados)
  }
  else
  {
    Serial.println("Error: AccZ es cero");
  }

  gyroRateRoll = GyroX / 65.5; // Tasa de giro en grados/segundo
  gyroRatePitch = GyroY / 65.5;
  RateYaw = GyroZ / 65.5;

  // Aplicar el filtro de Kalman
  kalmanUpdateRoll(accAngleRoll, gyroRateRoll, 0.01);
  kalmanUpdatePitch(accAnglePitch, gyroRatePitch);

  AngleRoll = x_roll[0];
  AnglePitch = x_pitch[0];
}

void setupMPU()
{
  Serial.begin(115200);
  Serial.println("Iniciando sensores...");

  // Inicializar el bus I2C para MPU6050 (pines 21 y 22)
  Wire.begin(SDA_MPU, SCL_MPU);
  Wire.setClock(400000);

  // Inicializar MPU6050
  Serial.println("Inicializando MPU6050...");
  delay(250);
  Wire.beginTransmission(0x68);
  delay(250);
  Wire.write(0x6B);
  Wire.write(0x00);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(250);
  Wire.endTransmission();
}

void loopMPU()
{

  Serial.print("Roll Angle=");
  Serial.print(AngleRoll);
  Serial.print("Pitch Angle=");
  Serial.println(AnglePitch);
  delay(50);
}
