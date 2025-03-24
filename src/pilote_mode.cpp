#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP32Servo.h>
#include <VL53L0X.h>
#include "variables.h"
#include "mpu.h"
#include "pilote_mode.h"

// === MATRIZ LQR PRECALCULADA (4x12) ===
// tau_x, tau_y, tau_z, thrust
float Klqr_att[4][12] = {
    // phi, theta, psi, p, q, r, int_phi, int_theta, phi_ref, theta_ref, psi_ref, 1
    {3.2, 0.0, 0.0, 1.8, 0.0, 0.0, 0.5, 0.0, -4.5, 0.0, 0.0, 0.0}, // tau_x
    {0.0, 3.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.4, 0.0, -4.2, 0.0, 0.0}, // tau_y
    {0},                                                           // tau_z (no usado aún)
    {0}                                                            // thrust (no usado aún)
};

// === VARIABLES DE CONTROL ===
extern float phi_ref, theta_ref, psi_ref;
extern float integral_phi, integral_theta;
float prev_error_phi = 0.0;
float prev_error_theta = 0.0;

// === SETUP INICIAL ===
void setup_pilote_mode()
{
  Serial.begin(115200);
  Serial.println("Iniciando modo pilote...");
  Wire.begin();
  pinMode(pinLed, OUTPUT);
  accelgyro.initialize();
  calibrateSensors();
  setupMotores_pilote();
  Serial.println("Setup completado.");
}

// === LOOP CON CONTROL LQR ===
void loop_pilote_mode()
{
  // Calcular errores
  float error_phi = phi_ref - AngleRoll;
  float error_theta = theta_ref - AnglePitch;

  // Actualizar términos integrales
  integral_phi = constrain(integral_phi + error_phi * 0.01, -50, 50);
  integral_theta = constrain(integral_theta + error_theta * 0.01, -50, 50);

  // Vector de estado x[12]
  float x[12] = {
      AngleRoll, AnglePitch, AngleYaw,
      RateRoll, RatePitch, RateYaw,
      integral_phi, integral_theta,
      phi_ref, theta_ref, psi_ref,
      1.0f // bias
  };

  // Aplicar LQR: u = -K * x
  float u[4] = {0};

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 12; j++)
    {
      u[i] -= Klqr_att[i][j] * x[j];
    }
  }

  float tau_x = u[0];
  float tau_y = u[1];

  applyControl(tau_x, tau_y);
}

// === CONTROL A LOS MOTORES ===
void applyControl(float tau_x, float tau_y)
{
  int pwm1 = 1500 - tau_x - tau_y;
  int pwm2 = 1500 - tau_x + tau_y;
  int pwm3 = 1500 + tau_x + tau_y;
  int pwm4 = 1500 + tau_x - tau_y;

  pwm1 = constrain(pwm1, 1200, 1800);
  pwm2 = constrain(pwm2, 1200, 1800);
  pwm3 = constrain(pwm3, 1200, 1800);
  pwm4 = constrain(pwm4, 1200, 1800);

  mot1.writeMicroseconds(pwm1);
  mot2.writeMicroseconds(pwm2);
  mot3.writeMicroseconds(pwm3);
  mot4.writeMicroseconds(pwm4);
}

// === CALIBRACIÓN DEL MPU6050 ===
void calibrateSensors()
{
  Serial.println("\nCalibrando sensores...");
  digitalWrite(pinLed, HIGH);

  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);

  meansensors();
  Serial.println("\nCalculando offsets...");
  calibration();

  accelgyro.setXAccelOffset(ax_offset);
  accelgyro.setYAccelOffset(ay_offset);
  accelgyro.setZAccelOffset(az_offset);
  accelgyro.setXGyroOffset(gx_offset);
  accelgyro.setYGyroOffset(gy_offset);
  accelgyro.setZGyroOffset(gz_offset);

  Serial.println("Calibración completada.");
  digitalWrite(pinLed, LOW);
}

void meansensors()
{
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  while (i < (buffersize + 101))
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (i > 100 && i <= (buffersize + 100))
    {
      buff_ax += ax;
      buff_ay += ay;
      buff_az += az;
      buff_gx += gx;
      buff_gy += gy;
      buff_gz += gz;
    }
    i++;
    delay(2);
  }

  mean_ax = buff_ax / buffersize;
  mean_ay = buff_ay / buffersize;
  mean_az = buff_az / buffersize;
  mean_gx = buff_gx / buffersize;
  mean_gy = buff_gy / buffersize;
  mean_gz = buff_gz / buffersize;
}

void calibration()
{
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;

  while (1)
  {
    int ready = 0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);
    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();

    if (abs(mean_ax) <= acel_deadzone)
      ready++;
    else
      ax_offset -= mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone)
      ready++;
    else
      ay_offset -= mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone)
      ready++;
    else
      az_offset += (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone)
      ready++;
    else
      gx_offset -= mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone)
      ready++;
    else
      gy_offset -= mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone)
      ready++;
    else
      gz_offset -= mean_gz / (giro_deadzone + 1);

    if (ready == 6)
      break;
  }
}

// === INICIALIZAR MOTORES ===
void setupMotores_pilote()
{
  mot1.attach(mot1_pin);
  mot2.attach(mot2_pin);
  mot3.attach(mot3_pin);
  mot4.attach(mot4_pin);

  Serial.println("Iniciando ESCs...");
  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);
  delay(2000);
}
