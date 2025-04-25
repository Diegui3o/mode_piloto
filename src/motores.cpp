#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP32Servo.h>
#include <piloto_mode.h>
#include "variables.h"
#include "mpu.h"

void setupMotores()
{
    Serial.begin(115200);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    mot1.attach(mot1_pin, 1000, 2000);
    mot2.attach(mot2_pin, 1000, 2000);
    mot3.attach(mot3_pin, 1000, 2000);
    mot4.attach(mot4_pin, 1000, 2000);

    // Inicializar ESCs
    mot1.writeMicroseconds(2000);
    mot2.writeMicroseconds(2000);
    mot3.writeMicroseconds(2000);
    mot4.writeMicroseconds(2000);
    delay(2000);

    mot1.writeMicroseconds(IDLE_PWM);
    mot2.writeMicroseconds(IDLE_PWM);
    mot3.writeMicroseconds(IDLE_PWM);
    mot4.writeMicroseconds(IDLE_PWM);
    delay(2000);
    Serial.println("Motores inicializados");
}

void encenderMotores(int speed)
{
    mot1.writeMicroseconds(speed);
    mot2.writeMicroseconds(speed);
    mot3.writeMicroseconds(speed);
    mot4.writeMicroseconds(speed);
}

void apagarMotores()
{
    mot1.writeMicroseconds(1000); // Apagar los motores
    mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000);
    mot4.writeMicroseconds(1000);
}

// === CONTROL A LOS MOTORES ===
void applyControl(float tau_x, float tau_y, float tau_z)
{
    float pwm1 = InputThrottle - tau_x - tau_y;
    float pwm2 = InputThrottle - tau_x + tau_y;
    float pwm3 = InputThrottle + tau_x + tau_y;
    float pwm4 = InputThrottle + tau_x - tau_y;

    // Limitar valores PWM
    MotorInput1 = constrain(pwm1, 1000, 2000);
    MotorInput2 = constrain(pwm2, 1000, 2000);
    MotorInput3 = constrain(pwm3, 1000, 2000);
    MotorInput4 = constrain(pwm4, 1000, 2000);

    // Enviar señales
    mot1.writeMicroseconds(round(MotorInput1));
    mot2.writeMicroseconds(round(MotorInput2));
    mot3.writeMicroseconds(round(MotorInput3));
    mot4.writeMicroseconds(round(MotorInput4));
}