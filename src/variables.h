#ifndef VARIABLES_H
#define VARIABLES_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <MPU6050.h>

extern MPU6050 accelgyro;
extern volatile float RatePitch, RateRoll, RateYaw;
extern float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw, AccXCalibration, AccYCalibration, AccZCalibration;
extern int pinLed;

extern int ESCfreq;

extern uint32_t LoopTimer;
extern float t;

extern volatile float AnglePitch_est;
extern volatile float AngleRoll_est;

extern Servo mot1;
extern Servo mot2;
extern Servo mot3;
extern Servo mot4;

extern const int mot1_pin;
extern const int mot2_pin;
extern const int mot3_pin;
extern const int mot4_pin;

// Variables de estado
extern float phi_ref, theta_ref, psi_ref; // Ángulos de referencia

// Variables de control
extern float integral_phi, integral_theta, integral_psi;

// Variables para la calibración
extern int buffersize;    // Cantidad de lecturas para promediar
extern int acel_deadzone; // Zona muerta del acelerómetro
extern int giro_deadzone; // Zona muerta del giroscopio

extern int16_t ax, ay, az, gx, gy, gz;
extern int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
extern int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

extern volatile float AngleRoll_est;
extern volatile float AnglePitch_est;
extern volatile float tau_x, tau_y, tau_z;
extern volatile float error_phi, error_theta, error_psi;

extern volatile uint32_t current_time;
extern volatile uint32_t last_channel_1;
extern volatile uint32_t last_channel_2;
extern volatile uint32_t last_channel_3;
extern volatile uint32_t last_channel_4;
extern volatile uint32_t last_channel_5;
extern volatile uint32_t last_channel_6;
extern volatile uint32_t timer_1;
extern volatile uint32_t timer_2;
extern volatile uint32_t timer_3;
extern volatile uint32_t timer_4;
extern volatile uint32_t timer_5;
extern volatile uint32_t timer_6;
extern volatile int ReceiverValue[6];
extern const int channel_1_pin;
extern const int channel_2_pin;
extern const int channel_3_pin;
extern const int channel_4_pin;
extern const int channel_5_pin;
extern const int channel_6_pin;

extern int ThrottleIdle;
extern int ThrottleCutOff;

// Kalman filters for angle mode
extern volatile float AccX, AccY, AccZ;
extern volatile float GyroXdps, GyroYdps, GyroZdps;
extern volatile float AngleRoll, AnglePitch, AngleYaw;
extern volatile float DesiredAngleRoll, DesiredAnglePitch;
extern volatile float ErrorAngleRoll, ErrorAnglePitch;
extern volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
extern volatile float PrevItermAngleRoll, PrevItermAnglePitch;

extern float complementaryAngleRoll;
extern float complementaryAnglePitch;

extern volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
// Estado estimado: [ángulo, sesgo]
extern volatile float x_roll[2];
extern volatile float x_pitch[2];

// Matriz de covarianza del error
extern float dt;      // Paso de tiempo (ajustar según la frecuencia de muestreo)
extern float Q_angle; // Covarianza del ruido del proceso (ángulo)
extern float Q_gyro;  // Covarianza del ruido del proceso (giroscopio)
extern float R_angle; // Covarianza del ruido de medición (acelerómetro)

// Estado y matrices de covarianza para roll
extern volatile float x_roll[2]; // [ángulo, bias_del_giroscopio]
extern float P_roll[2][2];       // Matriz de covarianza del error

// Estado y matrices de covarianza para pitch
extern volatile float x_pitch[2]; // [ángulo, bias_del_giroscopio]
extern float P_pitch[2][2];       // Matriz de covarianza del error

extern float dt;      // Paso de tiempo
extern float Q_angle; // Covarianza del ruido del proceso (ángulo)
extern float Q_gyro;  // Covarianza del ruido del proceso (giroscopio)
extern float R_angle; // Covarianza del ruido de medición (acelerómetro)

// Variables para el FFAKF
extern float C; // Covarianza del residual (roll)extern float lambda_roll;  // Factor de olvido (roll)
extern float lambda;

extern float accAngleRoll;  // Ángulo de roll (grados)
extern float accAnglePitch; // Ángulo de pitch (grados)
extern float gyroRateRoll;  // Tasa de giro en grados/segundo
extern float gyroRatePitch;

extern float accAngleY;
extern float accAngleX;

#endif // VARIABLES_H