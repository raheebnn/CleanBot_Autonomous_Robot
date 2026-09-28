# Hardware & wiring

## Motor drivers (BTS7960)

The motor brain uses `lgpio` on `gpiochip4` (Raspberry Pi 5). Pin numbers are BCM GPIO.

| Signal | Left driver | Right driver |
|---|---|---|
| RPWM (forward) | GPIO 12 | GPIO 18 |
| LPWM (reverse) | GPIO 13 | GPIO 19 |
| R_EN | GPIO 24 | GPIO 20 |
| L_EN | GPIO 25 | GPIO 21 |

- PWM frequency: 1 kHz. Duty is capped at 60 % in software (`MAX_DUTY`).
- Connect driver GND to Pi GND. Power the motors from a separate battery, never from the Pi's 5 V rail.
- To change pins, edit `L_*` / `R_*` at the top of `raspberry_pi/rpi5_motor_brain_slam.py`.

## IMU (MPU-6050)

| MPU-6050 | Raspberry Pi 5 |
|---|---|
| VCC | 3.3 V (pin 1) |
| GND | GND (pin 6) |
| SDA | GPIO 2 / SDA1 (pin 3) |
| SCL | GPIO 3 / SCL1 (pin 5) |

I²C bus 1, address `0x68`. Enable I²C with `sudo raspi-config` → Interface Options. Mount the IMU rigidly: the camera-to-IMU transform in `calibration/webcam_fixed_v3.yaml` assumes a 15° camera tilt and 0.04 m offset.

## Camera

Raspberry Pi Camera Module 3 on the CSI port, captured with `picamera2` at 640×480, 25 FPS.

## Ultrasonic (HC-SR04)

Physically mounted on a front servo. It is not yet read by the current navigation code (see roadmap). If you wire it, remember the ECHO pin outputs 5 V — use a voltage divider to 3.3 V.

## Power & safety

- Put a physical kill switch between the motor battery and the drivers.
- Lift the wheels off the ground for the first test run.
