#include <MPU6050.h> // Asegura que el compilador conoce la clase MPU6050
#include "variables.h"

MPU6050 accelgyro;

volatile float RatePitch = 0.0, RateRoll = 0.0, RateYaw = 0.0;
float RateCalibrationPitch = 0.0, RateCalibrationRoll = 0.0, RateCalibrationYaw = 0.0;
float AccXCalibration = 0.0, AccYCalibration = 0.0, AccZCalibration = 0.0;
int pinLed = 2;

int ESCfreq = 500;
float PAngleRoll = 1.2, PAnglePitch = 1.2;
float IAngleRoll = 0.1, IAnglePitch = 0.1;
float DAngleRoll = 0.04, DAnglePitch = 0.04;

float PRateRoll = 0.625;
float IRateRoll = 2.1;
float DRateRoll = 0.0088;

float PRatePitch = 0.625;
float IRatePitch = 2.1;
float DRatePitch = 0.0088;

float PRateYaw = 4.0;
float IRateYaw = 3.0;
float DRateYaw = 0.0;

volatile float AngleRoll_est;
volatile float AnglePitch_est;
volatile float tau_x, tau_y, tau_z;
volatile float error_phi, error_theta, error_psi;

int buffersize = 1000; // Cantidad de lecturas para promediar
int acel_deadzone = 8; // Zona muerta del acelerómetro
int giro_deadzone = 1; // Zona muerta del giroscopio

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

// Variables para la calibración
uint32_t LoopTimer;
float t = 0.004;

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;

const int mot1_pin = 13;
const int mot2_pin = 12;
const int mot3_pin = 14;
const int mot4_pin = 27;

int16_t ax, ay, az, gx, gy, gz;

volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0;
volatile uint32_t last_channel_2 = 0;
volatile uint32_t last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0;
volatile uint32_t last_channel_5 = 0;
volatile uint32_t last_channel_6 = 0;
volatile uint32_t timer_1;
volatile uint32_t timer_2;
volatile uint32_t timer_3;
volatile uint32_t timer_4;
volatile uint32_t timer_5;
volatile uint32_t timer_6;
volatile int ReceiverValue[6];
const int channel_1_pin = 34;
const int channel_2_pin = 35;
const int channel_3_pin = 32;
const int channel_4_pin = 33;
const int channel_5_pin = 25;
const int channel_6_pin = 26;

volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;
volatile float KalmanGainPitch;
volatile float KalmanGainRoll;

int ThrottleIdle = 1050;
int ThrottleCutOff = 1000;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[3] = {0, 0, 0};

// Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch, AngleYaw;
volatile float GyroXdps, GyroYdps, GyroZdps;
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// Variables de estado
float phi_ref = 0.0;
float theta_ref = 0.0;
float psi_ref = 0.0;
float integral_phi = 0.0;
float integral_theta = 0.0;
float integral_psi = 0.0;

// Estado estimado: [ángulo, sesgo]
float dt = 0.05;       // Paso de tiempo (ajustar según la frecuencia de muestreo)
float Q_angle = 0.001; // Covarianza del ruido del proceso (ángulo)
float Q_gyro = 0.003;  // Covarianza del ruido del proceso (giroscopio)
float R_angle = 0.03;  // Covarianza del ruido de medición (acelerómetro)

// Estado y matrices de covarianza para roll
volatile float x_roll[2] = {0, 0};     // [ángulo, bias_del_giroscopio]
float P_roll[2][2] = {{1, 0}, {0, 1}}; // Matriz de covarianza del error

// Estado y matrices de covarianza para pitch
volatile float x_pitch[2] = {0, 0};     // [ángulo, bias_del_giroscopio]
float P_pitch[2][2] = {{1, 0}, {0, 1}}; // Matriz de covarianza del error

// Variables para el FFAKF
float lambda = 1.0; // Forgetting factor
float C = 0.0;      // Measurement residual covariance

float accAngleRoll;  // Ángulo de roll (grados)
float accAnglePitch; // Ángulo de pitch (grados)
float gyroRateRoll;  // Tasa de giro en grados/segundo
float gyroRatePitch;