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
```

## Dependencies

To compile `main.cpp`, ensure the following libraries are installed in your Arduino IDE:

* **LSM6** - For Gyroscope/Accelerometer
* **LIS3MDL** - For Magnetometer
* **EZDist** - For Ultrasonic sensor management
* **Wire** - Standard Arduino I2C library

## Setup & Usage

### 1. Calibrate
*  Upload `calibration/magnetometer_calibration.ino` to the robot
*  Rotate the robot 360° to capture min/max values
*  Update `MAG_X_OFFSET` and `MAG_Y_OFFSET` in `main.cpp` with the new values

### 2. Test
*  Upload scripts from `unit-tests/` to verify motors turn correctly and sensors provide valid data

### 3. Run
*  Open `main.cpp`
*  Verify pin definitions (`TRIG_1`, `pwm1`, etc.) match your wiring
*  Upload to the board
*  The robot will start in `ORIGIN_CALIBRATION` mode (wait 2 seconds) before beginning its path

## Safety Mechanisms

* **Watchdog Timer:** If the main loop hangs for more than 1000ms, the WDT triggers `handleWatchdogTrigger()`, stopping all motors and resetting back to the `ORIGIN_CALIBRATION` state
