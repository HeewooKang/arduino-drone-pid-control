# Arduino Drone PID Control

Arduino-based quadcopter attitude control using an MPU6050 gyroscope sensor and PID control.

This project reads angular velocity data from an MPU6050 sensor through I2C, estimates the drone attitude by gyro integration, and controls four motors using PWM output.

---

## Project Overview

This project implements a basic PID-based attitude control system for a small quadcopter.

The controller receives throttle and direction commands through serial input, calculates roll, pitch, and yaw correction values, and mixes them into four motor PWM outputs.

The main control logic consists of:

* Gyroscope data reading from MPU6050
* Gyro offset calibration
* Angle estimation using gyro integration
* PID-based attitude correction
* Four-motor PWM mixing

---

## Demo Video

### Arduino Drone PID Control

[Watch Arduino drone PID demo](assets/arduino_drone_pid_demo.mp4)

This demo shows the Arduino-based drone PID control test using an MPU6050 gyroscope sensor and PWM motor output.

---

## Tech Stack

* Arduino
* C/C++
* MPU6050 gyroscope sensor
* I2C communication
* PWM motor control
* PID control

---

## Hardware Pin Configuration

| Component   | Pin         |
| ----------- | ----------- |
| Motor A     | D6          |
| Motor B     | D10         |
| Motor C     | D9          |
| Motor D     | D5          |
| MPU6050 SDA | Arduino SDA |
| MPU6050 SCL | Arduino SCL |
| MPU6050 VCC | 5V or 3.3V  |
| MPU6050 GND | GND         |

---

## Control Input

The drone receives commands through serial input.

| Input   | Function                   |
| ------- | -------------------------- |
| `0`     | Stop throttle              |
| `1 ~ 9` | Increase throttle          |
| `w`     | Forward                    |
| `x`     | Backward                   |
| `a`     | Move left                  |
| `d`     | Move right                 |
| `q`     | Yaw left                   |
| `e`     | Yaw right                  |
| `s`     | Hover / reset target angle |

---

## PID Control Structure

The controller uses P, I, and D control terms.

```text
PID Output = P Control + I Control + D Control
```

### P Control

P control corrects the current angle error.

```cpp
P = Kp * error
```

### I Control

I control corrects accumulated angle error.

```cpp
I = Ki * integral_error
```

### D Control

D control reduces angular velocity using gyroscope rate feedback.

```cpp
D = Kd * (-gyro_rate)
```

Since the target angle is mostly constant, the derivative of the angle error can be approximated using the negative gyro angular velocity.

---

## Motor Mixing

The PID correction values are mixed into four motor outputs.

```cpp
speedA = throttle + BalY + BalX + BalZ;
speedB = throttle - BalY + BalX - BalZ;
speedC = throttle - BalY - BalX + BalZ;
speedD = throttle + BalY - BalX - BalZ;
```

Each motor output is limited to the PWM range.

```cpp
constrain(speed, 0, 250);
```

---

## Source Code

Main Arduino file:

```text
arduino_drone_pid_controller.ino
```

---

## How to Use

1. Connect the MPU6050 sensor to the Arduino through I2C.
2. Connect the four motors or motor drivers to the PWM pins.
3. Open the `.ino` file in Arduino IDE.
4. Select the correct Arduino board and port.
5. Upload the code.
6. Open Serial Monitor.
7. Set the baud rate to `115200`.
8. Send throttle and direction commands.

Example inputs:

```text
1
2
3
w
a
s
0
```

---

## Important Notes

* Keep the drone stationary during startup calibration.
* The MPU6050 gyro offset is calculated using 1000 samples.
* This code estimates attitude using gyro integration only, so long-term drift can occur.
* The motor output uses `analogWrite()`.
* This code is suitable for PWM-based motor drivers.
* If BLDC motors with ESCs are used, the motor output part should be changed to servo-style PWM control.
* PID gains must be tuned experimentally before real flight.

---

## Future Improvements

* Add accelerometer-based angle correction
* Add complementary filter
* Add ESC calibration support
* Add wireless control
* Add safety cutoff logic
* Add PID gain tuning interface
* Add debug serial output
* Improve attitude estimation accuracy

---

## Author

Heewoo Kang
