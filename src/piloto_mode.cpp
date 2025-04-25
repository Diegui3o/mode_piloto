#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP32Servo.h>
#include <piloto_mode.h>
#include "variables.h"
#include <esp_task_wdt.h>
#include "mpu.h"
#include "motores.h"

// Define T as a global variable
float T = 0.0;

// Variable to track MPU calibration status
bool mpu_ready = false;

// === Matrices LQR ===
const float Ki_at[3][3] = {
    {19.1623, 0, 0},
    {0, 19.1623, 0},
    {0, 0, 3.87}};

const float Kc_at[3][6] = {
    {6.3882, 0, 0, 2.2, 0, 0},
    {0, 6.4022, 0, 0, 2.2, 0},
    {0, 0, 2.97864, 0, 0, 1.3182}};

// === Matrices LQR para altitud ===
const float Ki_alt = 31.6228;
const float Kc_alt[2] = {28.8910, 10.5624};

// === SETUP INICIAL ===
void setup_pilote_mode()
{
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, HIGH);
  delay(50);
  Serial.begin(115200);
  Serial.println("Iniciando modo pilote...");
  setupMotores();
  Serial.println("Setup completado.");
  digitalWrite(pinLed, LOW);
}

// === LOOP CON CONTROL LQR ===
void loop_pilote_mode()
{
  // Estado del sistema
  float x_c[6] = {AngleRoll, AnglePitch, AngleYaw, gyroRateRoll, gyroRatePitch, RateYaw};
  float x_i[3] = {integral_phi, integral_theta, integral_psi};

  // Actualizar integrales
  x_i[0] += error_phi * dt;
  x_i[1] += error_theta * dt;
  x_i[2] += error_psi * dt;

  error_phi = phi_ref - x_c[0];
  error_theta = theta_ref - x_c[1];
  error_psi = psi_ref - 0;

  // Control LQR
  tau_x = Ki_at[0][0] * integral_phi + Kc_at[0][0] * error_phi - Kc_at[0][3] * gyroRateRoll;
  tau_y = Ki_at[1][1] * integral_theta + Kc_at[1][1] * error_theta - Kc_at[1][4] * gyroRatePitch;
  tau_z = Ki_at[2][2] * integral_psi + Kc_at[2][2] * error_psi - Kc_at[2][5] * RateYaw;

  InputThrottle = 1500; // Empuje total calculado por el controlador de altitud
  applyControl(tau_x, tau_y, tau_z);
}
