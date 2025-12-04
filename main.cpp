// SYSC 4805 - L2 G7
// microcontroller code for automatic snowplow

#include <stdint.h>
// #include "arduino_code.h"
#include <math.h>
#include <Wire.h>  // I2C
#include <LSM6.h>  // gyroscope & accelerometer
#include <LIS3MDL.h>  // magnetometer
#include <EZDist.h>  // Ultrasonic sensors

/* State variables */
enum TopState { 
    ORIGIN_CALIBRATION,  // starting point
    FORWARD,
    BACKWARD,
    TURN_90_1,
    TURN_90_2,
    FORWARD_SHORT,
    TURN_NEXT_LAP,
    END_REACHED,
    SENSE_CORNER,
    NEXT_TRIAL_SETUP,  // go to appropriate corner depending on the trial
    TRAVEL_FROM_TOP_LEFT,
    TRAVEL_FROM_TOP_RIGHT,
    TRAVEL_FROM_BOT_LEFT,
    TRAVEL_FROM_BOT_RIGHT
};

enum CornerState { // corners
    TOP_LEFT,
    TOP_RIGHT,
    BOT_LEFT,
    BOT_RIGHT
};

enum DirectionState { // directions 
    RIGHT,
    DOWN,
    LEFT,
    UP
};

// START default 
DirectionState DIRECTION = RIGHT;
TopState currentState = ORIGIN_CALIBRATION;
TopState lastState;
CornerState lastCorner = TOP_LEFT;  // the origin
int numTrial = 1;
int numLap = 0;

/* Ultrasonic 1 */
#define TRIG_1 35  // sending pulse
#define ECHO_1 A7  // receiving pulse

/* Ultrasonic 2 */
#define TRIG_2 37  // sending pulse
#define ECHO_2 A6  // receiving pulse

long duration1 = 0;  // pulse duration for sensor 1
long duration2 = 0;  // pulse duration for sensor 2
float distance1 = 0;  // calculated cm distance from the object for sensor 1
float distance2 = 0;  // calculated cm distance from the object for sensor 2

/* IMU variables */
LSM6 imu;
bool imu_ready = false;
float currentYaw = 0.0;
char report[20];
const float GYRO_Z_BIAS = 0.5; // offset 

// non-blocking timer
unsigned long previousTime = 0;
const long IMU_INTERVAL = 10;  // milliseconds


/* Magnetometer */
LIS3MDL mag;
bool mag_ready = false;
const float FILTER_GAIN = 0.05;
const float MAG_X_OFFSET = -4081; // calibrated using the magnometer calibration script
const float MAG_Y_OFFSET = 2174; // calibrated using the magnometer calibration script


/* IR Sensor */
#define LEFT_IR 34
#define RIGHT_IR 36
int L_detected;
int R_detected;

/* Flags */
bool OBSTACLE_DETECTED = false;
bool PERP_LINE_DETECTED = false;
bool PARA_LINE_DETECTED = false;

/* Line sensor pins and threshold */
const int SENSOR_LS_LEFT = A0;
const int SENSOR_LS_CENTER = A1;
const int SENSOR_LS_RIGHT = A2;
const int SENSOR_RS_LEFT = A3;
const int SENSOR_RS_CENTER = A4;
const int SENSOR_RS_RIGHT = A5;
const int THRESHOLD = 150;

/* Line sensor readings */
int Vleft1, Vcenter1, Vright1; // Sensor 1 (left)
int Vleft2, Vcenter2, Vright2; // Sensor 2 (right)
int Vref1, Vref2, Vref3;

/* Motors pins */
const int dir1 = 13;
const int pwm1 = 12;
const int dir2 = 11;
const int pwm2 = 10;
const int dir3 = 9;
const int pwm3 = 8;
const int dir4 = 7;
const int pwm4 = 6;
const int SPEED = 100;
const int TURN_SPEED = 255;

/* Additional variables for state management */
unsigned long turnStartTime = 0;
const unsigned long TURN_DURATION = 1000;  // 1 second for 90 degrees
unsigned long backupStartTime = 0;
const unsigned long BACKUP_DURATION = 500;  // 0.5 seconds to backup
unsigned long shortMoveStartTime = 0;
const unsigned long SHORT_MOVE_DURATION = 1000;  // 1 second for short move
unsigned long travelStartTime = 0;
const unsigned long TRAVEL_DURATION = 2000;  // 2 seconds for travel
bool originCalibratedFlag = false;
bool endDetectedFlag = false;
bool obstacleDetectedFlag = false;
bool lineDetectedFlag = false;
bool backupCompleteFlag = false;
bool turnCompleteFlag = false;
bool shortMoveCompleteFlag = false;
bool turnLeftThisLap = false;
int turnCount = 0;
bool obstacleFlag = false;

/* turning variables */
float turnStartYaw = 0.0;
float turnTargetYaw = 0.0;
bool turningActive = false;
const float TURN_TOLERANCE = 3.0;  // degrees
float lastGyroZ = 0.0;  // latest gyro z (dps) from IMU
int turnDirectionSign = 0;  // +1 desired yaw increasing, -1 desired yaw decreasing
unsigned long turnTimeoutStart = 0;
const unsigned long TURN_IMU_TIMEOUT = 3000;  // ms, force-exit if stuck

/* Motor control methods */
void driveForward(int speed) { // go straight
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, HIGH);
    digitalWrite(dir3, HIGH);
    digitalWrite(dir4, HIGH);
    analogWrite(pwm1, speed);
    analogWrite(pwm2, speed);
    analogWrite(pwm3, speed);
    analogWrite(pwm4, speed);
}

void driveForwardShort(int speed) { // between 2 90 degree turns
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, HIGH);
    digitalWrite(dir3, HIGH);
    digitalWrite(dir4, HIGH);
    analogWrite(pwm1, speed);
    analogWrite(pwm2, speed);
    analogWrite(pwm3, speed);
    analogWrite(pwm4, speed);
}

void driveBackward(int speed) { // backup
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
    digitalWrite(dir3, LOW);
    digitalWrite(dir4, LOW);
    analogWrite(pwm1, speed);
    analogWrite(pwm2, speed);
    analogWrite(pwm3, speed);
    analogWrite(pwm4, speed);
}

void startTurn90FixedDirection() { // turn 90 degree alternating in direction(based on lap)
    if (turningActive) return;
    turnStartYaw = currentYaw;
    float delta = turnLeftThisLap ? +90.0f : -90.0f;
    turnTargetYaw = normalizeAngle(turnStartYaw + delta);
    turnDirectionSign = (delta > 0) ? +1 : -1;
    doTurnByDirection();
    turningActive = true;
    turnTimeoutStart = millis();
}

bool turn90CompleteIMU() { // completes the turn & makes adjustment based on yaw
    if (!turningActive) return false;
    // compute current error
    float err = angleError(turnTargetYaw, currentYaw);
    Serial.print("Turn error: ");
    Serial.println(err);
    // 1) If within tolerance => done
    if (abs(err) <= TURN_TOLERANCE) {
        stopMotors();
        turningActive = false;
        Serial.println("Turn completed (within tolerance).");
        return true;
    }
    // 2) If robot is rotating the wrong direction, fix it
    // lastGyroZ > 0 means yaw increasing (dps); <0 means yaw decreasing
    int actualSign = (lastGyroZ > 0.5f) ? +1 : ((lastGyroZ < -0.5f) ? -1 : 0);
    if (actualSign != 0 && actualSign != turnDirectionSign) {
        // we're rotating opposite to desired direction so reverse motors
        Serial.print("Detected wrong rotation direction (gyroZ=");
        Serial.print(lastGyroZ);
        Serial.println("). Reversing motors.");
        if (turnDirectionSign > 0) right(SPEED);
        else left(SPEED);
    }
    // 3) Ramp down speed as we approach target (reduces overshoot)
    float absErr = abs(err);
    int rampSpeed = TURN_SPEED;
    if (absErr < 20.0f) rampSpeed = max(30, (int)(TURN_SPEED * (absErr / 20.0f)));  // scale down
    if (turnDirectionSign > 0) right(rampSpeed);
    else left(rampSpeed);
    // 4) Timeout: if trying for too long, force exit to avoid stuck state
    if (millis() - turnTimeoutStart > TURN_IMU_TIMEOUT) {
        Serial.println("Turn timeout reached — forcing exit and stopping motors.");
        stopMotors();
        turningActive = false;
        return true;
    }
    return false;
}

/* Motor controls */
void right(int speed) { // only left 2 motors on 
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, HIGH);
    digitalWrite(dir3, LOW);
    digitalWrite(dir4, LOW);
    analogWrite(pwm1, speed);
    analogWrite(pwm2, speed);
    analogWrite(pwm3, speed);
    analogWrite(pwm4, speed);
}

void left(int speed) { // only right 2 motors on
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
    digitalWrite(dir3, HIGH);
    digitalWrite(dir4, HIGH);
    analogWrite(pwm1, speed);
    analogWrite(pwm2, speed);
    analogWrite(pwm3, speed);
    analogWrite(pwm4, speed);
}

void stopMotors() { // all motors off
    analogWrite(pwm1, 0);
    analogWrite(pwm2, 0);
    analogWrite(pwm3, 0);
    analogWrite(pwm4, 0);
}

/* Line sensor read */
void readLineSensors() {
    // avergae of left line sensor
    Vleft1 = analogRead(SENSOR_LS_LEFT);
    Vcenter1 = analogRead(SENSOR_LS_CENTER);
    Vright1 = analogRead(SENSOR_LS_RIGHT);
    Vref1 = (Vleft1 + Vright1 + Vcenter1) / 3;

    // avergae of right line sensor
    Vleft2 = analogRead(SENSOR_RS_LEFT);
    Vcenter2 = analogRead(SENSOR_RS_CENTER);
    Vright2 = analogRead(SENSOR_RS_RIGHT);
    Vref2 = (Vleft2 + Vcenter2 + Vright2) / 3;

    // average of both sensors
    Vref3 = (Vref1 + Vref2) / 2;
}

/* Reading Ultrasonic Sensor */
long rawUltrasonicCM(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);  // setting at low to begin with
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);  // sensor detection begin
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);  // sensors stop sending waiting for echo
    long duration = pulseIn(echoPin, HIGH, 25000);  // how long it stays high
    if (duration == 0) return 999;  // if no echo detected
    /* Convert to cm (speed of sound = 0.034 cm per microsecond) */
    return duration * 0.034 / 2;  // divide by 2 cause of round trip
}

/* Filter out invalid values */
long medianUltrasonic(int trigPin, int echoPin) {
    long a = rawUltrasonicCM(trigPin, echoPin);  // reading 1
    long b = rawUltrasonicCM(trigPin, echoPin);  // reading 2
    long c = rawUltrasonicCM(trigPin, echoPin);  // reading 3
    // sort a, b, c (so that b becomes the middle value)
    if (a > b) {
        long t = a;
        a = b;
        b = t;
    }
    if (b > c) {
        long t = b;
        b = c;
        c = t;
    }
    if (a > b) {
        long t = a;
        a = b;
        b = t;
    }
    long m = b;  // median
    // Filter out impossible distances
    if (m < 2 || m > 200) {
        return 999;
    }
    return m;
}

/* Ultrasonic read function */
void readUltrasonic() {
    // Update global distances
    distance1 = medianUltrasonic(TRIG_1, ECHO_1);
    distance2 = medianUltrasonic(TRIG_2, ECHO_2);
    // Set obstacle flag if something is close
    if (distance1 < 20 || distance2 < 20) {
        obstacleFlag = true;
    }

    // Debugging
    Serial.print("Ultrasonic - Sensor 1: ");
    Serial.print(distance1);
    Serial.print(" | Sensor 2: ");
    Serial.print(distance2);
    Serial.print(" | ObstacleFlag: ");
    Serial.println(obstacleFlag);
}

/* Clear Flag */
void resetObstacleFlag() {
    obstacleFlag = false;
}

/* IMU Sensor */
void readIMU() {
    // Safety check: if sensor isn't ready, do nothing
    if (!imu_ready) return;

    // Don't calculate drift if less than 10ms has passed
    // Without this, deltaTime can be 0.00, causing zero division or noise
    if (millis() - previousTime < IMU_INTERVAL) return; 

    unsigned long currentTime = millis();
    float deltaTime = (float)(currentTime - previousTime) / 1000.0;
    
    // Guard against huge time jumps
    if (deltaTime <= 0 || deltaTime > 0.5) deltaTime = 0.01;
    
    previousTime = currentTime;

    // GYROSCOPE
    imu.read();
    float Gz_dps = ((int16_t)imu.g.z) * 8.75f / 1000.0f;
    Gz_dps -= GYRO_Z_BIAS;
    lastGyroZ = Gz_dps;

    // Integrate Yaw
    currentYaw += Gz_dps * deltaTime;

    // MAGNETOMETER 
    if (mag_ready) {
        mag.read();

        // Apply Offsets
        float mag_x = (float)mag.m.x - MAG_X_OFFSET;
        float mag_y = (float)mag.m.y - MAG_Y_OFFSET;

        // Calculate Heading
        float magHeading = atan2(mag_y, mag_x) * 180.0 / PI;

        // Normalize to range (0 - 360)
        if (magHeading < 0) magHeading += 360.0;
        
        // If currentYaw is 0.0 (just turned on), it might be far from Magnetic North
        // This snaps them together instantly so the robot doesn't drift wildly at start
        static bool firstMagReading = true;
        if (firstMagReading) {
            currentYaw = magHeading;
            firstMagReading = false;
        } else {
            // Normal Fusion Logic
            float error = magHeading - currentYaw;

            // Shortest path logic
            if (error > 180) error -= 360;
            if (error < -180) error += 360;

            // Apply Filter
            currentYaw += (error * FILTER_GAIN);
        }
    }

    // NORMALIZATION
    currentYaw = fmod(currentYaw, 360.0);
    if (currentYaw < 0.0) currentYaw += 360.0;

    // Debugging
    Serial.print("Yaw: "); Serial.println(currentYaw);
}

/* IR Sensor */
void readIR() {
    // Left IR
    L_detected = digitalRead(LEFT_IR);
    // Right IR
    R_detected = digitalRead(RIGHT_IR);
}

/* Calibration */
void calibrateOrigin() {
    // Implement origin calibration logic (reads sensors and set initial position)
    static unsigned long calibrateStart = millis();
    if (millis() - calibrateStart > 2000) {  // 2 seconds calibration
        originCalibratedFlag = true;
        setInitialDirection();
    } 
}

/* Set Direction */
void setInitialDirection() {
    // Set DIRECTION based on numTrial
    if (numTrial == 1) {
        DIRECTION = RIGHT;  // Trial 1: TOP_LEFT facing RIGHT
    } else if (numTrial == 2) {
        DIRECTION = LEFT;  // Trial 2: TOP_RIGHT facing LEFT
    } else if (numTrial == 3) {
        DIRECTION = DOWN;  // Trial 3: TOP_LEFT facing DOWN
    } else if (numTrial == 4) {
        DIRECTION = UP;  // Trial 4: BOT_LEFT facing UP
    }
}

void doTurnByDirection() { // alternates between left and right turns each lap
    if (turnLeftThisLap) {
        left(TURN_SPEED);
    } else {
        right(TURN_SPEED);
    }
}

bool originCalibrated() {
    return originCalibratedFlag;
}

/* End Detection */
bool endDetected() {
    // if both line sensors are above the threshold
    return (Vref1 > 600 && Vref2 > 600);
}

/* Obstacle Detection */
bool obstacleDetected() {
    // if either the ultrasonic(less than 5cm range) or ir sees an obstacle
    if (turningActive) return false; // doesnt check when turning 
    return (distance1 < 5 || distance2 < 5 || !L_detected || !R_detected);
}

/* Line Detection */
bool lineDetected() {
    // if any line sensors are above the threshold
    if (turningActive) return false;
    return (Vref1 > 600 || Vref2 > 600 || Vref3 > 600);
}

/* Check Movement Completion */
bool backupComplete() {
    return (millis() - backupStartTime > BACKUP_DURATION);
}
bool turnComplete() {
    return (millis() - turnStartTime > TURN_DURATION);
}
bool shortMoveComplete() {
    return (millis() - shortMoveStartTime > SHORT_MOVE_DURATION);
}
bool travelComplete() {
    return (millis() - travelStartTime > TRAVEL_DURATION);
}

/* Line following */
void forwardFollowLine(DirectionState ROBOT_SIDE) {
    // Implement line following logic
    // adjust motors based on sensor readings
    readLineSensors();
    if (ROBOT_SIDE == LEFT) {
        if (Vleft1 > THRESHOLD && Vright1 < THRESHOLD) {
            left(SPEED);
        } else if (Vright1 > THRESHOLD && Vleft1 < THRESHOLD) {
            right(SPEED);
        } else {
            driveForward(SPEED);
        }
    } else {
        if (Vleft2 > THRESHOLD && Vright2 < THRESHOLD) {
            left(SPEED);
        } else if (Vright2 > THRESHOLD && Vleft2 < THRESHOLD) {
            right(SPEED);
        } else {
            driveForward(SPEED);
        }
    }
}

/* Corener Detection */
CornerState detectCorner() {
    // uses yaw to find current corner
    if (currentYaw > 315 || currentYaw < 45) return TOP_LEFT; 
    else if (currentYaw > 45 && currentYaw < 135) return TOP_RIGHT;
    else if (currentYaw > 135 && currentYaw < 225) return BOT_RIGHT;
    else return BOT_LEFT;
}

/* Main Loop */
void loop() {
    // Read sensors periodically
    readIR();
    readUltrasonic();
    if (imu_ready) {
        readIMU();
    }
    readLineSensors();

    // Deugging 
    Serial.print("Left Line Sensor: ");
    Serial.print(Vref1);
    Serial.print(" | Right Line Sensor: ");
    Serial.print(Vref2);
    Serial.print(" | Avg Line Sensors: ");
    Serial.println(Vref3);

    // WDT
    feedWatchdog();
    if (checkWatchdog()) {
        handleWatchdogTrigger();
    }

    // State Machine
    switch (currentState) {
    case ORIGIN_CALIBRATION: // calibrate origin
        Serial.println("In case 'ORIGIN_CALIBRATION': start calibrating");
        calibrateOrigin();
        // when done, move forward
        if (originCalibrated()) {
            Serial.println("In case 'ORIGIN_CALIBRATION': done calibrating");
            currentState = FORWARD;
        }
        break;
    case FORWARD: // move forward
        Serial.println("In case 'FORWARD'");
        driveForward(SPEED);
        if (obstacleDetected() || lineDetected()) {
            stopMotors();
            turnCount = 0;  // start fresh
            currentState = TURN_90_1;  // always go to first turn
        } else if (endDetected()) {
            Serial.println("In case 'FORWARD': end detected");
            stopMotors();
            numLap = 0;
            currentState = END_REACHED;
        }
        break;
    case BACKWARD: // move backward
        Serial.println("In case 'BACKWARD'");
        driveBackward(SPEED);
        if (backupComplete()) {
            Serial.println("In case 'BACKWARD': backup complete");
            stopMotors();
            currentState = TURN_90_1;
        }
        break;
    case TURN_90_1: // 1st 90 degree turn
        if (!turningActive) {
            startTurn90FixedDirection();
        }
        if (turn90CompleteIMU()) {
            turnCount = 1;
            currentState = FORWARD_SHORT;
        }
        break;
    case FORWARD_SHORT: // small forward movemnet before 2nd 90 degree turn
        Serial.println("In case 'FORWARD_SHORT'");
        driveForwardShort(SPEED);
        if (shortMoveComplete()) {
            Serial.println("In case 'FORWARD_SHORT': short move complete");
            stopMotors();
            currentState = TURN_90_2;
        }
        break;
    case TURN_90_2: // second 90 degree turn
        if (!turningActive) {
            startTurn90FixedDirection();
        }
        if (turn90CompleteIMU()) {
            turnCount = 2;  // Both turns complete so swicth direction
            turnLeftThisLap = !turnLeftThisLap;
            currentState = FORWARD;
        }
        break;
    case END_REACHED: // after end of trial go to next trial(from different corner)
        Serial.println("In case 'END_REACHED'");
        // use line sensor to follow the boundary line to the corner
        if (numTrial == 1) {
            Serial.println("In case 'END_REACHED': trial 1");
            if (DIRECTION == LEFT) {
                Serial.println("In case 'END_REACHED': direction left");
                forwardFollowLine(LEFT);
            } else if (DIRECTION == RIGHT) {
                Serial.println("In case 'END_REACHED': direction right");
                forwardFollowLine(RIGHT);
            }
        } else if (numTrial == 2) {
            Serial.println("In case 'END_REACHED': trial 2");
            if (DIRECTION == LEFT) {
                Serial.println("In case 'END_REACHED': direction left");
                forwardFollowLine(LEFT);
            } else if (DIRECTION == RIGHT) {
                Serial.println("In case 'END_REACHED': direction right");
                forwardFollowLine(RIGHT);
            }
        } else if (numTrial == 3) {
            Serial.println("In case 'END_REACHED': trial 3");
            if (DIRECTION == UP) {
                Serial.println("In case 'END_REACHED': direction up");
                forwardFollowLine(RIGHT);
            } else if (DIRECTION == DOWN) {
                Serial.println("In case 'END_REACHED': direction down");
                forwardFollowLine(LEFT);
            }
        } else if (numTrial == 4) {
            Serial.println("In case 'END_REACHED': trial 4");
            if (DIRECTION == LEFT) {
                Serial.println("In case 'END_REACHED': direction left");
                forwardFollowLine(LEFT);
            } else if (DIRECTION == RIGHT) {
                Serial.println("In case 'END_REACHED': direction right");
                forwardFollowLine(RIGHT);
            }
        }
        if (lineDetected()) {
            Serial.println("In case 'END_REACHED': line detected");
            stopMotors();
            currentState = SENSE_CORNER;
        }
        break;
    case SENSE_CORNER: // check corner
        Serial.println("In case 'SENSE_CORNER'");
        lastCorner = detectCorner();
        if (lastCorner == TOP_LEFT) {
            Serial.println("In case 'SENSE_CORNER': TRAVEL_FROM_TOP_LEFT");
            currentState = TRAVEL_FROM_TOP_LEFT;
        } else if (lastCorner == TOP_RIGHT) {
            Serial.println("In case 'SENSE_CORNER': TRAVEL_FROM_TOP_RIGHT");
            currentState = TRAVEL_FROM_TOP_RIGHT;
        } else if (lastCorner == BOT_LEFT) {
            Serial.println("In case 'SENSE_CORNER': TRAVEL_FROM_BOT_LEFT");
            currentState = TRAVEL_FROM_BOT_LEFT;
        } else if (lastCorner == BOT_RIGHT) {
            Serial.println("In case 'SENSE_CORNER': TRAVEL_FROM_BOT_RIGHT");
            currentState = TRAVEL_FROM_BOT_RIGHT;
        }
        break;
    case TRAVEL_FROM_TOP_LEFT: // trial strating from top left corner
        Serial.println("In case 'TRAVEL_FROM_TOP_LEFT'");
        Serial.println("Travelling from top left...");
        travelStartTime = millis();
        if (numTrial == 1) {  // Go to TOP_RIGHT for trial 2
            Serial.println("In case 'TRAVEL_FROM_TOP_LEFT': trial 1");
            forwardFollowLine(LEFT);  // Follow top line
            if (travelComplete()) {
                Serial.println("In case 'TRAVEL_FROM_TOP_LEFT': trial 1 complete");
                stopMotors();
                numTrial = 2;
                currentState = ORIGIN_CALIBRATION;
            }
        } else if (numTrial == 3) {  // Go to BOT_LEFT for trial 4
            Serial.println("In case 'TRAVEL_FROM_TOP_LEFT': trial 4");
            forwardFollowLine(RIGHT);  // Follow left line
            if (travelComplete()) {
                Serial.println("In case 'TRAVEL_FROM_TOP_LEFT': trial 4 complete");
                stopMotors();
                numTrial = 4;
                currentState = ORIGIN_CALIBRATION;
            }
        } else if (numTrial == 7) {  // we go to BOT_LEFT
            Serial.println("In case 'TRAVEL_FROM_TOP_LEFT': trial 7");
            forwardFollowLine(RIGHT);
            if (travelComplete()) {
                Serial.println("In case 'TRAVEL_FROM_TOP_LEFT': trial 7 complete");
                stopMotors();
                numTrial++;
                currentState = ORIGIN_CALIBRATION;
            }
        }
        break;
    case TRAVEL_FROM_TOP_RIGHT: // trial strating from top right corner
        Serial.println("In case 'TRAVEL_FROM_TOP_RIGHT'");
        Serial.println("Travelling from top right...");
        travelStartTime = millis();
        if (numTrial == 2) {  // we go to TOP_LEFT
            Serial.println("In case 'TRAVEL_FROM_TOP_RIGHT': trial 2");
            forwardFollowLine(RIGHT);
            if (travelComplete()) {
                Serial.println("In case 'TRAVEL_FROM_TOP_RIGHT': trial 2 complete");
                stopMotors();
                numTrial++;
                currentState = ORIGIN_CALIBRATION;
            }
        } else if (numTrial == 6) {  // we go to TOP_LEFT
            Serial.println("In case 'TRAVEL_FROM_TOP_RIGHT': trial 6");
            forwardFollowLine(RIGHT);
            if (travelComplete()) {
                Serial.println("In case 'TRAVEL_FROM_TOP_RIGHT': trial 6 complete");
                stopMotors();
                numTrial++;
                currentState = ORIGIN_CALIBRATION;
            }
        }
        break;
    case TRAVEL_FROM_BOT_LEFT: // trial starting from bottom left corner
        Serial.println("In case 'TRAVEL_FROM_BOT_LEFT'");
        Serial.println("Travelling from bottom left...");
        travelStartTime = millis();
        if (numTrial == 4) {  // we go to BOT_RIGHT
            Serial.println("In case 'TRAVEL_FROM_BOT_LEFT': trial 4");
            forwardFollowLine(RIGHT);
            if (travelComplete()) {
                Serial.println("In case 'TRAVEL_FROM_BOT_LEFT': trial 4 complete");
                stopMotors();
                numTrial++;
                currentState = ORIGIN_CALIBRATION;
            }
        } else if (numTrial == 8) {  // we go to BOT_RIGHT
            Serial.println("In case 'TRAVEL_FROM_BOT_LEFT': trial 8");
            forwardFollowLine(RIGHT);
            if (travelComplete()) {
                Serial.println("In case 'TRAVEL_FROM_BOT_LEFT': trial 8 complete");
                stopMotors();
                numTrial++;
                currentState = ORIGIN_CALIBRATION;
            }
        }
        break;
    case TRAVEL_FROM_BOT_RIGHT: // trial starting from bottom right corner
        Serial.println("In case 'TRAVEL_FROM_BOT_RIGHT'");
        Serial.println("Travelling from bottom right...");
        travelStartTime = millis();
        if (numTrial == 5) {  // we go to TOP_RIGHT
            Serial.println("In case 'TRAVEL_FROM_BOT_RIGHT': trial 5");
            forwardFollowLine(RIGHT);
            if (travelComplete()) {
                Serial.println("In case 'TRAVEL_FROM_BOT_RIGHT': trial 5 complete");
                stopMotors();
                numTrial++;
                currentState = ORIGIN_CALIBRATION;
            }
        }
        break;
    default: // unknown state
        Serial.println("In case 'default'");
        Serial.println("Unknown state!");
        break;
}
delay(20);  // small loop delay
}

/* Setup */
void setup() {
    Serial.begin(9600);

    /* Motor drivers */
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);
    pinMode(dir3, OUTPUT);
    pinMode(dir4, OUTPUT);
    pinMode(pwm1, OUTPUT);
    pinMode(pwm2, OUTPUT);
    pinMode(pwm3, OUTPUT);
    pinMode(pwm4, OUTPUT);
    Serial.println("Motor drivers ready!");

    /* IMU sensors */
    // over I2C protocol(SDA and SCL)
    previousTime = millis();  // setup non-blocking timer
    Wire.begin();
    if (imu.init()) {
        Serial.println("IMU sensors ready!");
        imu.enableDefault();
        imu_ready = true;  // Set the flag to TRUE
    } else {
        Serial.println("WARNING: IMU failed to initialize. Continuing without it.");
        imu_ready = false;  // Set the flag to FALSE
    }
    
	/* Magnetometer sensors */
	if (mag.init()) {
        Serial.println("Magnetometer sensors ready!");
        mag.enableDefault();
        mag_ready = true;
    } else {
        Serial.println("WARNING: Magnetometer failed to initialize.");
        mag_ready = false;
    }

    /* IR Obstacle Detection Sensor */
    pinMode(LEFT_IR, INPUT);
    pinMode(RIGHT_IR, INPUT);
    Serial.println("IR sensors ready!");

    /* Ultrasonic sensors */
    pinMode(TRIG_1, OUTPUT);
    pinMode(ECHO_1, INPUT);
    pinMode(TRIG_2, OUTPUT);
    pinMode(ECHO_2, INPUT);
}

/* Watchdog Timer */
unsigned long lastFeedTime = 0;
const unsigned long WATCHDOG_TIMEOUT = 1000;  // milliseconds

void feedWatchdog() {
    lastFeedTime = millis();
}

// to check the system is alive
bool checkWatchdog() {
    return (millis() - lastFeedTime) > WATCHDOG_TIMEOUT;
}

// handling crashes
void handleWatchdogTrigger() {
    // if WDT detects crash, then stop motors and go back to start state
    Serial.println("Watchdog Triggered");
    stopMotors();
    calibrateOrigin();
    lastFeedTime = millis();  // reset the timer
}

/* IMU helpers */
float normalizeAngle(float a) { // normalize angle within range (0 - 360)
    while (a >= 360.0) a -= 360.0;
    while (a < 0.0) a += 360.0;
    return a;
}

float angleError(float target, float current) { // calculate angle error
    float err = target - current;
    if (err > 180.0) err -= 360.0;
    if (err < -180.0) err += 360.0;
    return err;
}
