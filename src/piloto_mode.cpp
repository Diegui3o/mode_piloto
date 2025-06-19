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

// Variable to track MPU calibration status
bool mpu_ready = false;

// === Función de saturación para modo deslizante ===
float sat(float value, float threshold)
{
    if (value > threshold)
        return 1.0;
    else if (value < -threshold)
        return -1.0;
    else
        return value / threshold;
}

// === Matrices LQR ===
const float Ki_at[3][3] = {
    {17, 0, 0},
    {0, 17, 0},
    {0, 0, 0.1}};

const float Kc_at[3][6] = {
    {5.2, 0, 0, 2.5, 0, 0},
    {0, 5.2, 0, 0, 2.5, 0},
    {0, 0, 5.3, 0, 0, 1.6}};

// === SETUP INICIAL ===
void setup_pilote_mode()
{
    pinMode(pinLed, OUTPUT);
    Serial.begin(115200);
    Serial.println("Iniciando modo pilote...");
    InputThrottle = 1000;
    delay(100);
    Serial.println("Setup completado.");
}

void loop_pilote_mode(float dt)
{
    // Inicializar throttle solo una vez al principio
    static bool throttle_initialized = false;
    if (!throttle_initialized)
    {
        InputThrottle = 1000; // Empezar en 1000
        throttle_initialized = true;
    }

    // 2. SEGUNDO: Estado del sistema
    float x_c[6] = {AngleRoll, AnglePitch, AngleYaw, gyroRateRoll, gyroRatePitch, RateYaw};

    // 3. TERCERO: Calcular errores
    error_phi = phi_ref - x_c[0];
    error_theta = theta_ref - x_c[1];
    error_psi = psi_ref - x_c[2];

    // 4. CUARTO: Actualizar integrales con saturación específica por eje
    integral_phi += error_phi * dt;
    integral_theta += error_theta * dt;
    integral_psi += error_psi * dt;

    // 5. QUINTO: Control LQR usando las integrales actualizadas
    tau_x = Ki_at[0][0] * integral_phi + Kc_at[0][0] * error_phi - Kc_at[0][3] * x_c[3] - N * x_c[4] * x_c[5];
    tau_y = Ki_at[1][1] * integral_theta + Kc_at[1][1] * error_theta - Kc_at[1][4] * x_c[4] - N * x_c[3] * x_c[5];
    tau_z = Ki_at[2][2] * integral_psi + Kc_at[2][2] * error_psi - Kc_at[2][5] * x_c[5] - N * x_c[4] * x_c[3];

    // Incrementar throttle gradualmente de 1000 a 1850
    if (InputThrottle < 1850)
    {
        InputThrottle += 3.0; // Incremento de 3 unidades por ciclo
        if (InputThrottle > 1850)
        {
            InputThrottle = 1850; // Limitar a máximo 1850
        }
    } // Aplicar control cuando el throttle esté por encima del mínimo de seguridad
    if (InputThrottle > 1020)
    {
        applyControl(tau_x, tau_y, tau_z);
    }
    else
    {
        applyControl(0, 0, 0);
        apagarMotores();
        // Resetear integrales cuando el throttle está bajo
        integral_phi = integral_theta = integral_psi = 0;
    }
}