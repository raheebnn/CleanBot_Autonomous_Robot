#!/usr/bin/env python3
"""
MPU6050 six-position calibration utility for Raspberry Pi.

The user is asked to place the IMU in six approximately static orientations:
+X, -X, +Y, -Y, +Z, -Z.

The accelerometer is expected to measure approximately +/- 1 g on the
gravity axis. The script estimates an affine correction:

    a_corrected = M @ a_raw + b

and a stationary gyroscope bias.

Output:
    imu_calibration.json

Requirements:
    pip install smbus2
"""

import json
import math
import time
from pathlib import Path

from smbus2 import SMBus

BUS_ID = 1
MPU_ADDR = 0x68
OUTPUT = Path(__file__).with_name("imu_calibration.json")

ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B

# Default full-scale settings:
# accelerometer: +/-2 g
# gyroscope: +/-250 deg/s
ACCEL_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0

def twos_complement(high, low):
    value = (high << 8) | low
    if value & 0x8000:
        value -= 65536
    return value

def read_vector(bus, register):
    data = bus.read_i2c_block_data(MPU_ADDR, register, 6)
    return [
        twos_complement(data[0], data[1]),
        twos_complement(data[2], data[3]),
        twos_complement(data[4], data[5]),
    ]

def read_accel_g(bus):
    raw = read_vector(bus, ACCEL_XOUT_H)
    return [v / ACCEL_LSB_PER_G for v in raw]

def read_gyro_dps(bus):
    raw = read_vector(bus, GYRO_XOUT_H)
    return [v / GYRO_LSB_PER_DPS for v in raw]

def average_samples(bus, seconds=5.0, sample_period=0.01):
    accel = []
    gyro = []
    end = time.time() + seconds
    while time.time() < end:
        accel.append(read_accel_g(bus))
        gyro.append(read_gyro_dps(bus))
        time.sleep(sample_period)

    def mean(rows):
        return [sum(r[i] for r in rows) / len(rows) for i in range(3)]

    return mean(accel), mean(gyro)

def solve_3x3(A, b):
    """Simple Gaussian elimination for a 3x3 linear system."""
    aug = [A[i][:] + [b[i]] for i in range(3)]

    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) < 1e-12:
            raise ValueError("Singular matrix during calibration.")
        aug[col], aug[pivot] = aug[pivot], aug[col]

        p = aug[col][col]
        aug[col] = [x / p for x in aug[col]]

        for r in range(3):
            if r == col:
                continue
            factor = aug[r][col]
            aug[r] = [aug[r][c] - factor * aug[col][c] for c in range(4)]

    return [aug[i][3] for i in range(3)]

def least_squares_3x3(X, y):
    """Solve (X^T X) beta = X^T y for one output axis."""
    xtx = [[0.0] * 3 for _ in range(3)]
    xty = [0.0] * 3

    for row, target in zip(X, y):
        for i in range(3):
            xty[i] += row[i] * target
            for j in range(3):
                xtx[i][j] += row[i] * row[j]

    return solve_3x3(xtx, xty)

def fit_affine(accel_measurements):
    """
    Fit each corrected axis independently from measured six-position data.

    For each axis:
        corrected_axis = c0*x + c1*y + c2*z

    Targets are the ideal gravity vectors.
    """
    targets = [
        [ 1, 0, 0],
        [-1, 0, 0],
        [ 0, 1, 0],
        [ 0,-1, 0],
        [ 0, 0, 1],
        [ 0, 0,-1],
    ]

    # Add a bias by first estimating per-axis offset from opposite faces,
    # then fit a 3x3 scale/cross-axis matrix around the bias.
    bias = []
    for axis in range(3):
        pairs = [
            (accel_measurements[0][axis], accel_measurements[1][axis]),
            (accel_measurements[2][axis], accel_measurements[3][axis]),
            (accel_measurements[4][axis], accel_measurements[5][axis]),
        ]
        bias.append(sum((p + n) / 2.0 for p, n in pairs) / 3.0)

    corrected = [[v[i] - bias[i] for i in range(3)]
                 for v in accel_measurements]

    matrix = []
    for out_axis in range(3):
        y = [targets[k][out_axis] for k in range(6)]
        matrix.append(least_squares_3x3(corrected, y))

    return bias, matrix

def main():
    print("MPU6050 six-position calibration")
    print("Keep the IMU stationary during each measurement.")
    print("Do not move the sensor until the sampling phase is complete.\n")

    with SMBus(BUS_ID) as bus:
        bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
        bus.write_byte_data(MPU_ADDR, ACCEL_CONFIG, 0x00)
        bus.write_byte_data(MPU_ADDR, GYRO_CONFIG, 0x00)
        time.sleep(1)

        labels = [
            "+X face upward",
            "-X face upward",
            "+Y face upward",
            "-Y face upward",
            "+Z face upward",
            "-Z face upward",
        ]

        accel_measurements = []
        gyro_measurements = []

        for label in labels:
            input(f"Place the IMU with {label}, then press Enter...")
            print("Collecting samples for 5 seconds...")
            a, g = average_samples(bus)
            accel_measurements.append(a)
            gyro_measurements.append(g)
            print("  accel:", ["%.5f" % x for x in a])
            print("  gyro :", ["%.5f" % x for x in g])
            print()

        accel_bias, accel_matrix = fit_affine(accel_measurements)

        gyro_bias = [
            sum(row[i] for row in gyro_measurements) / len(gyro_measurements)
            for i in range(3)
        ]

        result = {
            "sensor": "MPU6050",
            "bus": BUS_ID,
            "i2c_address": hex(MPU_ADDR),
            "accelerometer": {
                "bias_g": accel_bias,
                "correction_matrix": accel_matrix,
                "units": "g",
            },
            "gyroscope": {
                "bias_dps": gyro_bias,
                "units": "deg/s",
            },
            "notes": [
                "Apply a_corrected = correction_matrix @ (a_raw_g - bias_g).",
                "Subtract gyro bias from raw gyro measurements.",
                "Recalibrate if the IMU mounting, sensor, or operating temperature changes substantially."
            ]
        }

        OUTPUT.write_text(json.dumps(result, indent=2))
        print(f"\nCalibration saved to: {OUTPUT}")
        print("\nGyroscope bias (deg/s):", gyro_bias)
        print("Accelerometer bias (g):", accel_bias)
        print("\nDone.")

if __name__ == "__main__":
    main()
