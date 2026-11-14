// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2
//
// Integrated with the supplied robot sketch. Keep your existing
// controller_callbacks.h and its implementation in the project.
// A: follow the black line. X: original color-return run. B: stop while held;
// release B to return to manual driving. R1/L1: scoop in every mode.
//
// DEFAULT: the two wired sensor outputs on GPIO32/33 from your current code.
// For reliable corner detection, wire all eight QTR-8A outputs and select 1.
// Edit the eight-pin list to match PHYSICAL LEFT-TO-RIGHT order before use.
// Two sensors cannot reliably distinguish a 90-degree corner from a lost line.
// Their centered position must put BOTH selected sensors over the black line.
//
// Motor 1 is assumed LEFT and Motor 2 RIGHT. This version uses PWM on the
// existing direction inputs: the motor driver must support that arrangement.
// Use driveMotors()/driveStop() for ALL motor output, including callbacks;
// do not leave other digitalWrite() calls active on these PWM-controlled pins.
// Confirm QTR outputs are 3.3 V compatible with the ESP32. If the QTR is powered
// at 5 V, do not connect its analog outputs directly without conditioning.
//
// Host-tested with mocked APIs: 56 checks for 2 sensors, 78 for 8 sensors.
// Actual ESP32 compilation, wiring verification and track tuning remain needed.

#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>
#include <uni.h>
#include <QTRSensors.h>
#include "controller_callbacks.h"
#include <Wire.h>
#include <Arduino_APDS9960.h>
#include <ESP32SharpIR.h>
#include <ESP32Servo.h>
#include <ESP32PWM.h> // Part of ESP32Servo; coordinates PWM allocation with Scoop.
#include <math.h>

#define APDS9960_INT 0
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 100000
#define Motor1Pin1 18
#define Motor1Pin2 5
#define Motor2Pin1 17
#define Motor2Pin2 16

#ifndef USE_FULL_QTR_ARRAY
#define USE_FULL_QTR_ARRAY 0
#endif

#if USE_FULL_QTR_ARRAY
// EXAMPLE original-ESP32 wiring: change to your actual connections.
// GPIO25/26 are ADC2; do not enable Wi-Fi with this example mapping.
const uint8_t LINE_PINS[] = {36, 39, 34, 35, 32, 33, 25, 26};
#else
const uint8_t LINE_PINS[] = {32, 33};
#endif
constexpr uint8_t SENSOR_COUNT = sizeof(LINE_PINS) / sizeof(LINE_PINS[0]);

// -------- Settings to tune on the actual robot --------
constexpr int BASE_SPEED = 110;       // PWM scale: 0..255.
constexpr int CURVE_SPEED = 80;
constexpr int TURN_SPEED = 90;
constexpr int MAX_LINE_SPEED = 170;
constexpr int MANUAL_SPEED = 140;
constexpr int COLOR_SPEED = 100;
constexpr bool INVERT_LEFT = false;  // Change if a wheel spins backward.
constexpr bool INVERT_RIGHT = false;
constexpr uint16_t BLACK_THRESHOLD = 500; // Calibrated scale: 0 white, 1000 black.
constexpr uint16_t MIN_CALIBRATION_SPAN = 80; // Raw 10-bit readings.
constexpr float KP = 110.0f;         // Error is normalized to -1..+1.
constexpr float KD = SENSOR_COUNT == 8 ? 0.8f : 0.0f;
constexpr uint32_t LINE_SAMPLE_MS = 10;
constexpr uint32_t CORNER_CONFIRM_MS = 20;
constexpr uint32_t CORNER_ADVANCE_MS = 120; // TUNE for sensor-to-axle distance!
constexpr uint32_t MIN_TURN_MS = 80;  // Guard against immediately finding old line.
constexpr uint32_t TURN_TIMEOUT_MS = 1800;
constexpr uint32_t GAP_COAST_MS = 60;
constexpr uint32_t SEARCH_TIMEOUT_MS = 1500;
constexpr uint8_t LINE_CONFIRM_SAMPLES = 3;
constexpr int COLOR_TOLERANCE = 7;    // Same raw RGB tolerance as your original.
constexpr uint8_t COLOR_CONFIRM_SAMPLES = 3;
constexpr uint32_t COLOR_TIMEOUT_MS = 1500;

extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];
TwoWire I2C_0 = TwoWire(0);
APDS9960 apds = APDS9960(I2C_0, APDS9960_INT);
Servo Scoop;
QTRSensors qtr;
uint16_t sensors[SENSOR_COUNT] = {};
ESP32PWM motorPwm[4];
ESP32SharpIR FrontSensor(ESP32SharpIR::GP2Y0A21YK0F, 12);
ESP32SharpIR LeftSensor(ESP32SharpIR::GP2Y0A21YK0F, 14);
ESP32SharpIR RightSensor(ESP32SharpIR::GP2Y0A21YK0F, 27);
int r = 0, g = 0, b = 0, a = 0;
bool lineReady = false;
bool colorReady = false;
ControllerPtr drivingController = nullptr;
bool previousA = false;
bool previousX = false;

enum class DriveMode { Manual, Line, Color, Stopped };
DriveMode driveMode = DriveMode::Manual;
enum class LineState { Following, Advancing, Turning, Searching, Lost };
struct LineData {
    LineState state = LineState::Following;
    uint32_t lastSample = 0, stateStarted = 0, missingSince = 0, candidateSince = 0;
    float error = 0, previousError = 0;
    int lastDirection = 0, turnDirection = 0, candidateDirection = 0;
    uint8_t blackCount = 0, stableCount = 0;
    bool missing = false, leftOldLine = false;
} lineData;
enum class ColorPhase { Capture, Leave, Return };
struct ColorData {
    ColorPhase phase = ColorPhase::Capture;
    int startR = 0, startG = 0, startB = 0;
    uint8_t stableCount = 0;
    uint32_t lastReading = 0;
} colorData;

// Explicit declaration keeps Arduino's generated prototypes from putting this
// custom-enum parameter before the LineState definition.
void beginPivot(LineState state, int direction, uint32_t now);

int limitSpeed(int value, int low, int high) {
    return value < low ? low : (value > high ? high : value);
}

void writeMotor(ESP32PWM& forward, ESP32PWM& reverse, int speed) {
    if (speed >= 0) {
        reverse.writeScaled(0);
        forward.writeScaled(speed / 255.0);
    } else {
        forward.writeScaled(0);
        reverse.writeScaled(-speed / 255.0);
    }
}

void driveMotors(int left, int right) {
    left = limitSpeed(left, -255, 255);
    right = limitSpeed(right, -255, 255);
    if (INVERT_LEFT) left = -left;
    if (INVERT_RIGHT) right = -right;
    writeMotor(motorPwm[0], motorPwm[1], left);
    writeMotor(motorPwm[2], motorPwm[3], right);
}

void driveStop() { driveMotors(0, 0); }
void driveForward() { driveMotors(COLOR_SPEED, COLOR_SPEED); }

void setupMotors() {
    const uint8_t pins[] = {Motor1Pin1, Motor1Pin2, Motor2Pin1, Motor2Pin2};
    for (uint8_t i = 0; i < 4; ++i) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
        motorPwm[i].attachPin(pins[i], 2000, 8);
        motorPwm[i].writeScaled(0);
    }
}

void handleConnectedController(ControllerPtr ctl) {
    onConnectedController(ctl); // Retain your existing controller-list logic.
}

void handleDisconnectedController(ControllerPtr ctl) {
    if (ctl == drivingController) {
        driveStop();
        driveMode = DriveMode::Stopped;
        drivingController = nullptr;
        previousA = previousX = false;
    }
    onDisconnectedController(ctl);
}

void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "DPAD: %d A: %d B: %d X: %d Y: %d LX: %d LY: %d RX: %d RY: %d L1: %d R1: %d L2: %d R2: %d\n",
        ctl->dpad(), ctl->a(), ctl->b(), ctl->x(), ctl->y(),
        ctl->axisX(), ctl->axisY(), ctl->axisRX(), ctl->axisRY(),
        ctl->l1(), ctl->r1(), ctl->l2(), ctl->r2());
}

void scoopServo(Controller* controller) {
    if (controller && controller->isConnected()) {
        if (controller->r1()) Scoop.write(0);
        if (controller->l1()) Scoop.write(180);
    }
}

void moveNoob(Controller* controller) {
    if (!controller || !controller->isConnected()) { driveStop(); return; }
    int forward = -controller->axisY();
    int turn = controller->axisRX();
    if (abs(forward) < 50) forward = 0;
    if (abs(turn) < 50) turn = 0;
    int left = forward + turn;
    int right = forward - turn;
    const int largest = abs(left) > abs(right) ? abs(left) : abs(right);
    if (largest > 512) { left = left * 512 / largest; right = right * 512 / largest; }
    driveMotors(left * MANUAL_SPEED / 512, right * MANUAL_SPEED / 512);
}

// Configure and calibrate ONCE, while motors are stopped.
bool lineSensor() {
    driveStop();
    qtr.setTypeAnalog();
    qtr.setSensorPins(LINE_PINS, SENSOR_COUNT);
    qtr.setSamplesPerSensor(4);
    for (uint8_t pin : LINE_PINS) analogSetPinAttenuation(pin, ADC_11db);
    // The original QTR-8A's emitter-control input is assumed left enabled.
    Console.printf("Sweep ALL %d sensors over black AND white during calibration.\n", SENSOR_COUNT);
    // QTR's analog calibration expects 0..1023. Restore 12 bits for Sharp IR later.
    analogReadResolution(10);
    for (uint16_t i = 0; i < 250; ++i) {
        qtr.calibrate();
        BP32.update();
        delay(20);
    }
    analogReadResolution(12);
    if (!qtr.calibrationOn.minimum || !qtr.calibrationOn.maximum) return false;
    bool okay = true;
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        const int span = int(qtr.calibrationOn.maximum[i]) - int(qtr.calibrationOn.minimum[i]);
        Console.printf("Sensor %d calibration span: %d\n", i, span);
        if (span < MIN_CALIBRATION_SPAN) okay = false;
    }
    return okay;
}

bool seesBlack(uint8_t index) { return sensors[index] >= BLACK_THRESHOLD; }
bool centerSeesBlack() {
    return seesBlack(SENSOR_COUNT / 2 - 1) || seesBlack(SENSOR_COUNT / 2);
}

bool lineIsCentered() {
    if (SENSOR_COUNT == 2) {
        return seesBlack(0) && seesBlack(1) && fabsf(lineData.error) <= 0.35f;
    }
    return centerSeesBlack() && !seesBlack(0) && !seesBlack(SENSOR_COUNT - 1) &&
           lineData.blackCount <= 3 && fabsf(lineData.error) <= 0.20f;
}

void readLineSensors() {
    analogReadResolution(10);
    const uint16_t position = qtr.readLineBlack(sensors);
    analogReadResolution(12);
    const float midpoint = (SENSOR_COUNT - 1) * 500.0f;
    lineData.error = (float(position) - midpoint) / midpoint; // -1 left, +1 right.
    lineData.blackCount = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        if (seesBlack(i)) ++lineData.blackCount;
    }
}

void failLineRun(const char* reason) {
    lineData.state = LineState::Lost;
    driveStop();
    Console.printf("Line run stopped: %s. Press A again, or B for manual.\n", reason);
}

void startLineRun() {
    driveStop();
    driveMode = DriveMode::Line;
    lineData = LineData{};
    if (!lineReady) { failLineRun("calibration failed"); return; }
    readLineSensors();
    if (lineData.blackCount == 0) { failLineRun("place sensors over the line"); return; }
    lineData.previousError = lineData.error;
    lineData.lastDirection = lineData.error < -0.15f ? -1 : (lineData.error > 0.15f ? 1 : 0);
    lineData.lastSample = uint32_t(millis());
    Console.printf("Line following started with %d sensors.\n", SENSOR_COUNT);
}

void steerAlongLine(uint32_t elapsedMs) {
    const float seconds = elapsedMs > 0 ? elapsedMs / 1000.0f : 0.001f;
    const float derivative = (lineData.error - lineData.previousError) / seconds;
    const float correction = KP * lineData.error + KD * derivative;
    lineData.previousError = lineData.error;
    const int base = fabsf(lineData.error) > 0.35f ? CURVE_SPEED : BASE_SPEED;
    driveMotors(limitSpeed(int(base + correction), 0, MAX_LINE_SPEED),
                limitSpeed(int(base - correction), 0, MAX_LINE_SPEED));
}

void pivot(int direction) { driveMotors(direction * TURN_SPEED, -direction * TURN_SPEED); }

void beginPivot(LineState state, int direction, uint32_t now) {
    lineData.state = state;
    lineData.stateStarted = now;
    lineData.turnDirection = direction;
    lineData.leftOldLine = !centerSeesBlack();
    lineData.stableCount = 0;
    pivot(direction);
}

// One control step. No delay() or blocking while loops here.
void line() {
    if (lineData.state == LineState::Lost) return;
    const uint32_t now = uint32_t(millis());
    const uint32_t elapsed = now - lineData.lastSample;
    if (elapsed < LINE_SAMPLE_MS) return;
    lineData.lastSample = now;
    readLineSensors();

    if (lineData.state == LineState::Advancing) {
        if (now - lineData.stateStarted >= CORNER_ADVANCE_MS) {
            beginPivot(LineState::Turning, lineData.turnDirection, now);
        } else {
            driveMotors(CURVE_SPEED, CURVE_SPEED);
        }
        return;
    }

    if (lineData.state == LineState::Turning || lineData.state == LineState::Searching) {
        const bool turning = lineData.state == LineState::Turning;
        const uint32_t timeout = turning ? TURN_TIMEOUT_MS : SEARCH_TIMEOUT_MS;
        if (now - lineData.stateStarted >= timeout) { failLineRun("line not recovered"); return; }
        if (!centerSeesBlack()) lineData.leftOldLine = true;
        const bool canFinish = !turning || (lineData.leftOldLine &&
                                           now - lineData.stateStarted >= MIN_TURN_MS);
        if (canFinish && lineIsCentered()) ++lineData.stableCount;
        else lineData.stableCount = 0;
        if (lineData.stableCount >= LINE_CONFIRM_SAMPLES) {
            lineData.state = LineState::Following;
            lineData.missing = false;
            lineData.candidateDirection = 0;
            lineData.previousError = lineData.error;
            steerAlongLine(elapsed);
        } else {
            pivot(lineData.turnDirection);
        }
        return;
    }

    // Check actual values: readLineBlack() returns a last-side position on white.
    if (lineData.blackCount == 0) {
        lineData.candidateDirection = 0;
        if (!lineData.missing) { lineData.missing = true; lineData.missingSince = now; }
        if (now - lineData.missingSince < GAP_COAST_MS) {
            driveMotors(CURVE_SPEED, CURVE_SPEED);
        } else if (lineData.lastDirection != 0) {
            beginPivot(LineState::Searching, lineData.lastDirection, now);
        } else {
            failLineRun("no last-known turn direction");
        }
        return;
    }
    if (lineData.missing) lineData.previousError = lineData.error;
    lineData.missing = false;

    // The full array provides enough spatial information for corner detection.
    // Two black sensors alone do not identify a corner or a junction.
    if (SENSOR_COUNT == 8) {
        if (seesBlack(0) && seesBlack(SENSOR_COUNT - 1)) {
            // Ambiguous crossing/broad patch: chosen policy is straight slowly.
            lineData.candidateDirection = 0;
            lineData.previousError = lineData.error;
            driveMotors(CURVE_SPEED, CURVE_SPEED);
            return;
        }
        const bool leftCorner = seesBlack(0) && seesBlack(1) && centerSeesBlack() &&
                                !seesBlack(SENSOR_COUNT - 1);
        const bool rightCorner = seesBlack(SENSOR_COUNT - 2) && seesBlack(SENSOR_COUNT - 1) &&
                                 centerSeesBlack() && !seesBlack(0);
        const int direction = leftCorner ? -1 : (rightCorner ? 1 : 0);
        if (direction != 0) {
            lineData.lastDirection = direction;
            if (lineData.candidateDirection != direction) {
                lineData.candidateDirection = direction;
                lineData.candidateSince = now;
            }
            if (now - lineData.candidateSince >= CORNER_CONFIRM_MS) {
                lineData.state = LineState::Advancing;
                lineData.stateStarted = now;
                lineData.turnDirection = direction;
            }
            lineData.previousError = lineData.error;
            driveMotors(CURVE_SPEED, CURVE_SPEED);
            return;
        }
    }
    lineData.candidateDirection = 0;
    if (lineData.error < -0.15f) lineData.lastDirection = -1;
    if (lineData.error > 0.15f) lineData.lastDirection = 1;
    steerAlongLine(elapsed);
}

// Nonblocking: false means no usable reading yet. Global r/g/b/a remain available.
bool getColor() {
    if (!colorReady || !apds.colorAvailable()) return false;
    return apds.readColor(r, g, b, a);
}

void startColorRun() {
    driveStop();
    colorData = ColorData{};
    colorData.lastReading = uint32_t(millis());
    driveMode = colorReady ? DriveMode::Color : DriveMode::Stopped;
    Console.printf(colorReady ? "Color run started.\n" : "Color sensor initialization failed.\n");
}

// Same purpose as the original: drive straight until the initial RGB returns.
// Three consecutive readings confirm departure/return instead of one noisy sample.
void colorRun() {
    const uint32_t now = uint32_t(millis());
    if (!getColor()) {
        if (now - colorData.lastReading >= COLOR_TIMEOUT_MS) {
            driveStop();
            driveMode = DriveMode::Stopped;
            Console.printf("Color run stopped: no sensor readings.\n");
        }
        return;
    }
    colorData.lastReading = now;
    if (colorData.phase == ColorPhase::Capture) {
        colorData.startR = r; colorData.startG = g; colorData.startB = b;
        colorData.phase = ColorPhase::Leave;
        Console.printf("Starting RGB: %d %d %d\n", r, g, b);
        driveForward();
        return;
    }
    const bool matches = abs(r - colorData.startR) < COLOR_TOLERANCE &&
                         abs(g - colorData.startG) < COLOR_TOLERANCE &&
                         abs(b - colorData.startB) < COLOR_TOLERANCE;
    const bool desiredReading = colorData.phase == ColorPhase::Leave ? !matches : matches;
    if (desiredReading) ++colorData.stableCount;
    else colorData.stableCount = 0;
    if (colorData.stableCount >= COLOR_CONFIRM_SAMPLES) {
        colorData.stableCount = 0;
        if (colorData.phase == ColorPhase::Leave) {
            colorData.phase = ColorPhase::Return;
        } else {
            driveStop();
            driveMode = DriveMode::Stopped;
            Console.printf("Starting color detected again: stopped.\n");
            return;
        }
    }
    driveForward();
}

void setup() {
    Scoop.attach(23);
    setupMotors();
    BP32.setup(&handleConnectedController, &handleDisconnectedController);
    BP32.forgetBluetoothKeys(); // Retained from your sketch; erases keys on each boot.
    esp_log_level_set("gpio", ESP_LOG_ERROR);
    uni_bt_allowlist_set_enabled(true); // Retains your existing allowlist configuration.
    I2C_0.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    apds.setInterruptPin(APDS9960_INT);
    colorReady = apds.begin();
    lineReady = lineSensor(); // The only line calibration call.
    Console.printf(lineReady ? "Line calibration OK.\n" : "Line calibration failed; sweep all sensors and reset.\n");
    FrontSensor.setFilterRate(1.0f);
    LeftSensor.setFilterRate(1.0f);
    RightSensor.setFilterRate(1.0f);
    Console.printf("A: line following. X: color run. B: stop/manual.\n");
}

void loop() {
    vTaskDelay(1);
    BP32.update();
    ControllerPtr ctl = nullptr;
    for (auto candidate : myControllers) {
        if (candidate && candidate->isConnected() && candidate->isGamepad()) {
            ctl = candidate;
            break; // One controller owns the motors.
        }
    }
    if (!ctl) {
        driveStop();
        driveMode = DriveMode::Stopped;
        drivingController = nullptr;
        previousA = previousX = false;
        return;
    }
    if (ctl != drivingController) {
        driveStop();
        driveMode = DriveMode::Manual;
        previousA = previousX = false;
        drivingController = ctl;
    }

    if (ctl->hasData()) {
        const bool aPressed = ctl->a();
        const bool xPressed = ctl->x();
        if (!ctl->b()) {
            if (aPressed && !previousA) startLineRun();
            else if (xPressed && !previousX) startColorRun();
        }
        previousA = aPressed;
        previousX = xPressed;
        scoopServo(ctl);
    }

    if (ctl->b()) {
        driveStop(); // No later joystick command can overwrite this stop.
        driveMode = DriveMode::Manual;
        return;
    }
    switch (driveMode) {
        case DriveMode::Manual: moveNoob(ctl); break;
        case DriveMode::Line: line(); break;
        case DriveMode::Color: colorRun(); break;
        case DriveMode::Stopped: driveStop(); break;
    }
}
