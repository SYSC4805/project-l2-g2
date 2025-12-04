# SYSC 4805 - Group L2-G7: Automatic Snowplow Robot

This repository contains the code for an autonomous snowplow robot designed for SYSC 4805. The system utilizes a Finite State Machine (FSM) to handle navigation, obstacle avoidance, line following, and self-calibration.

## Overview

The robot operates autonomously to clear an area by navigating in a lawmower pattern. It relies on sensor fusion (LSM6 Gyroscope + LIS3MDL Magnetometer) for precise turning and heading, while Ultrasonic and IR sensors provide obstacle detection.

### Key Features
* **Sensor Fusion:** Combines Gyroscope and Magnetometer data for drift-free heading (Yaw) calculation
* **Finite State Machine:** Robust logic handling states like `ORIGIN_CALIBRATION`, `FORWARD`, `TURN_90`, and `END_REACHED`
* **Crash Recovery:** Implements a Watchdog Timer (WDT) to reset the system safely in case of a freeze
* **Line Following:** Uses multiple line sensor arrays to detect boundaries and navigate between trials(next corner)

## Project Structure

This project is organized into the following active directories:

```text
project-l2-g2/
├── main.cpp                     # Main application logic (State Machine & Control Loop)
├── sensors                      # OLD: not used !
│   └── linedetection.ino 
├── calibration/                 # Sensor calibration scripts
│   └── magnetometer_calibration.ino
└── unit-tests/                  # Isolated tests for hardware components
    ├── IMUtest/                 # Gyroscope and Magnetometer tests
    ├── IRObstacleAvoidanceSensor.ino # IR tests
    ├── LineFollowingSensor.ino  # Line deetection tests 
    ├── MotorControl/            # Basic motor direction/speed checks
    ├── MotorIntegration.ino     # Motor Integration tests
    ├── UltrasonicSensor.ino     # Ultrasonic tests
    └── WDTtest/                 # Watchdog Timer logic simulation
