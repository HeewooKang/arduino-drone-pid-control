#include <Wire.h>

#define CONTROL_SERIAL Serial
#define MPU_ADDR 0x68

// Motor PWM pins
#define MOTOR_A 6
#define MOTOR_B 10
#define MOTOR_C 9
#define MOTOR_D 5

int throttle = 0;

double GyXOff = 0.0;
double GyYOff = 0.0;
double GyZOff = 0.0;

double AngleX = 0.0;
double AngleY = 0.0;
double AngleZ = 0.0;

double targetAngleX = 0.0;
double targetAngleY = 0.0;
double targetAngleZ = 0.0;

double integralX = 0.0;
double integralY = 0.0;
double integralZ = 0.0;

unsigned long t_prev = 0;

// PID gains
double Kp = 1.0;    // P control: angle error correction
double Ki = 0.45;   // I control: accumulated error correction
double Kd = 0.5;    // D control: angular velocity damping

void setup() {
  CONTROL_SERIAL.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  calibrateGyro();

  t_prev = micros();
}

void calibrateGyro() {
  int32_t GyXSum = 0;
  int32_t GyYSum = 0;
  int32_t GyZSum = 0;

  for (int i = 0; i < 1000; i++) {
    int16_t GyX, GyY, GyZ;
    readGyroRaw(GyX, GyY, GyZ);

    GyXSum += GyX;
    GyYSum += GyY;
    GyZSum += GyZ;

    delay(1);
  }

  GyXOff = GyXSum / 1000.0;
  GyYOff = GyYSum / 1000.0;
  GyZOff = GyZSum / 1000.0;
}

void readGyroRaw(int16_t &GyX, int16_t &GyY, int16_t &GyZ) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t GyXH = Wire.read();
  int16_t GyXL = Wire.read();
  int16_t GyYH = Wire.read();
  int16_t GyYL = Wire.read();
  int16_t GyZH = Wire.read();
  int16_t GyZL = Wire.read();

  GyX = (GyXH << 8) | GyXL;
  GyY = (GyYH << 8) | GyYL;
  GyZ = (GyZH << 8) | GyZL;
}

void readUserInput() {
  while (CONTROL_SERIAL.available() > 0) {
    char userInput = CONTROL_SERIAL.read();

    if (userInput >= '0' && userInput <= '9') {
      throttle = (userInput - '0') * 25;
    }
    else if (userInput == 'a') {
      targetAngleY = -10.0;   // left
    }
    else if (userInput == 'd') {
      targetAngleY = 10.0;    // right
    }
    else if (userInput == 'w') {
      targetAngleX = -10.0;   // forward
    }
    else if (userInput == 'x') {
      targetAngleX = 10.0;    // backward
    }
    else if (userInput == 'q') {
      targetAngleZ = -10.0;   // yaw left
    }
    else if (userInput == 'e') {
      targetAngleZ = 10.0;    // yaw right
    }
    else if (userInput == 's') {
      targetAngleX = 0.0;
      targetAngleY = 0.0;
      targetAngleZ = 0.0;     // hover
    }
  }
}

void loop() {
  readUserInput();

  int16_t GyXRaw, GyYRaw, GyZRaw;
  readGyroRaw(GyXRaw, GyYRaw, GyZRaw);

  double GyXD = GyXRaw - GyXOff;
  double GyYD = GyYRaw - GyYOff;
  double GyZD = GyZRaw - GyZOff;

  // Convert raw gyro value to deg/s
  double GyXR = GyXD / 131.0;
  double GyYR = GyYD / 131.0;
  double GyZR = GyZD / 131.0;

  unsigned long t_now = micros();
  double dt = (t_now - t_prev) / 1000000.0;
  t_prev = t_now;

  if (dt <= 0.0 || dt > 0.1) {
    dt = 0.001;
  }

  // Estimate angle by gyro integration
  AngleX += GyXR * dt;
  AngleY += GyYR * dt;
  AngleZ += GyZR * dt;

  // Reset when throttle is zero
  if (throttle == 0) {
    AngleX = 0.0;
    AngleY = 0.0;
    AngleZ = 0.0;

    integralX = 0.0;
    integralY = 0.0;
    integralZ = 0.0;
  }

  double errorX = targetAngleX - AngleX;
  double errorY = targetAngleY - AngleY;
  double errorZ = targetAngleZ - AngleZ;

  // =========================
  // PID Control
  // =========================

  // P control
  double P_X = Kp * errorX;
  double P_Y = Kp * errorY;
  double P_Z = Kp * errorZ;

  // I control
  integralX += errorX * dt;
  integralY += errorY * dt;
  integralZ += errorZ * dt;

  integralX = constrain(integralX, -50.0, 50.0);
  integralY = constrain(integralY, -50.0, 50.0);
  integralZ = constrain(integralZ, -50.0, 50.0);

  double I_X = Ki * integralX;
  double I_Y = Ki * integralY;
  double I_Z = Ki * integralZ;

  // D control
  // derivative of error ≈ -gyro angular velocity
  double D_X = Kd * (-GyXR);
  double D_Y = Kd * (-GyYR);
  double D_Z = Kd * (-GyZR);

  double BalX = P_X + I_X + D_X;
  double BalY = P_Y + I_Y + D_Y;
  double BalZ = P_Z + I_Z + D_Z;

  if (throttle == 0) {
    BalX = 0.0;
    BalY = 0.0;
    BalZ = 0.0;
  }

  // Motor mixing
  double speedA = throttle + BalY + BalX + BalZ;
  double speedB = throttle - BalY + BalX - BalZ;
  double speedC = throttle - BalY - BalX + BalZ;
  double speedD = throttle + BalY - BalX - BalZ;

  int iSpeedA = constrain((int)speedA, 0, 250);
  int iSpeedB = constrain((int)speedB, 0, 250);
  int iSpeedC = constrain((int)speedC, 0, 250);
  int iSpeedD = constrain((int)speedD, 0, 250);

  analogWrite(MOTOR_A, iSpeedA);
  analogWrite(MOTOR_B, iSpeedB);
  analogWrite(MOTOR_C, iSpeedC);
  analogWrite(MOTOR_D, iSpeedD);
}
