// SPDX-License-Identifier: Apache-2.0
// Original starter attribution: Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2
//
// ROBOTATHON: line following, two-color navigation, and manual driving.
// Keep controller_callbacks.h/.cpp and your existing Bluepad32 build project.
// This file replaces arduino_main.cpp; it is not a standalone ESP-IDF project.
//
// Controls: A = line, X = color, Y = calibrate line sensors, B = stop/manual.
// Manual: left stick Y = throttle, right stick X = steering; R1/L1 = scoop.
// After B or reconnecting, center both sticks before manual driving is armed.
// Calibration runs at boot, with the motors OFF. Sweep BOTH QTR outputs over
// black AND white until the serial monitor says calibration has passed.
//
// IMPORTANT PHYSICAL ASSUMPTIONS:
// * Motor 1 is left, motor 2 right; adjust inversion below if necessary.
// * QTR outputs are ordered physically left-to-right, ahead of the wheels.
//   Use two nearby elements so a centered line is visible to at least one.
// * The first two color tiles are straight ahead and have different, saturated
//   colors. Neutral floor/gaps are rejected. White/gray/black tiles require a
//   different floor classifier; lowering MIN_CHROMA alone can learn the floor.
// * Color A marks turning points; color B indicates the outgoing direction.
//   Start on A, learn B, drive to A, scan for B, repeat. No turn list is stored.
// * The downward color sensor MUST be ahead of the pivot axis: a turn must
//   physically sweep the sensor over the next B tile. A single APDS9960 cannot
//   see a tile elsewhere on the field. Tune the mounting and scan timing.
// * No unique finish marker is specified. Missing line / no B after an A causes
//   a bounded search and STOP, with an ambiguous-end diagnostic, not a claimed
//   successful finish. Test final-tile behavior on your actual course.

#include <Arduino.h>
#include <Arduino_APDS9960.h>
#include <Bluepad32.h>
#include <ESP32PWM.h>  // Bundled with ESP32Servo; shares its PWM allocator.
#include <ESP32Servo.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <esp_log.h>
#include <math.h>
#include <uni.h>

#include <cstdint>

#include "controller_callbacks.h"

extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// All wiring, speeds, and tuning values live here. Speeds use 0..255 PWM.
namespace Config {
constexpr uint8_t LEFT_IN1 = 18, LEFT_IN2 = 5;
constexpr uint8_t RIGHT_IN1 = 17, RIGHT_IN2 = 16;
constexpr bool INVERT_LEFT = false, INVERT_RIGHT = false;
constexpr float LEFT_TRIM = 1.0f, RIGHT_TRIM = 1.0f;
constexpr uint8_t LINE_PINS[] = {32, 33};
constexpr uint8_t LINE_COUNT = sizeof(LINE_PINS) / sizeof(LINE_PINS[0]);
constexpr uint8_t SERVO_PIN = 23, SDA_PIN = 21, SCL_PIN = 22;
constexpr int COLOR_INT_PIN = -1;              // No interrupt wire needed for polling.
constexpr int SCOOP_DOWN = 0, SCOOP_UP = 180;  // Tune to avoid mechanism binding.
constexpr int SCOOP_START = 90;
constexpr uint32_t MOTOR_PWM_HZ = 20000;
constexpr int DEADZONE = 60, MANUAL_MAX = 210;
constexpr uint32_t CONTROLLER_TIMEOUT_MS = 1000;  // Stop before Bluetooth detects a lost link.

constexpr uint16_t CALIBRATION_SAMPLES = 250, CALIBRATION_MIN_SPAN = 120;
constexpr uint32_t CALIBRATION_INTERVAL_MS = 20, LINE_INTERVAL_MS = 10;
constexpr uint16_t LINE_VISIBLE = 250;  // Calibrated: white=0, black=1000.
constexpr int LINE_SPEED = 105, LINE_SLOW_SPEED = 65, LINE_TURN_SPEED = 85;
constexpr float LINE_KP = 140.0f, LINE_KD = 45.0f;
constexpr uint32_t LINE_GAP_MS = 60, LINE_SWEEP_MS = 350;
constexpr uint8_t LINE_REACQUIRE_SAMPLES = 2;

constexpr int COLOR_SPEED = 75, COLOR_TURN_SPEED = 75;
constexpr uint8_t COLOR_LEARN_SAMPLES = 4, COLOR_MATCH_SAMPLES = 2;
constexpr float COLOR_MATCH_DISTANCE = 0.10f;  // Euclidean distance in RGB/sum.
constexpr float COLOR_MIN_SEPARATION = 0.24f;  // A and B must be clearly apart.
constexpr float MIN_CHROMA = 0.12f;            // max(R,G,B)/sum - min(R,G,B)/sum.
constexpr int COLOR_MIN_CLEAR = 20, COLOR_SATURATION = 60000;
constexpr uint32_t COLOR_POLL_MS = 10, COLOR_SENSOR_TIMEOUT_MS = 1500;
constexpr uint32_t COLOR_FIRST_TIMEOUT_MS = 5000, COLOR_TRAVEL_TIMEOUT_MS = 8000;
constexpr uint32_t COLOR_SETTLE_MS = 150;
// At the chosen turn speed, tune this to about 100 degrees per side. Keep the
// scan below 150 degrees so it does not rediscover the previous B behind you.
// Time is only an estimate of angle; wheel slip and battery level affect it.
constexpr uint32_t COLOR_SCAN_HALF_MS = 750;
// Optional forward adjustment to put the axle at a corner. Default: none.
constexpr uint32_t COLOR_PIVOT_ADVANCE_MS = 0;
constexpr uint32_t AUTONOMOUS_TIMEOUT_MS = 120000;
constexpr bool TELEMETRY = true;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 250;
}  // namespace Config

static_assert(Config::LINE_COUNT == 2, "This follower is tuned for two QTR outputs.");

enum class Mode { STOPPED,
                  MANUAL,
                  CALIBRATING,
                  LINE,
                  COLOR };
enum class ColorPhase {
    LEARN_FIRST,
    LEARN_SECOND,
    DRIVE_TO_FIRST,
    ADVANCE_AT_FIRST,
    SETTLE_AT_FIRST,
    SCAN_LEFT,
    RETURN_FROM_LEFT,
    SCAN_RIGHT,
    RETURN_FROM_RIGHT,
    SETTLE_ON_SECOND
};

TwoWire colorBus(0);
APDS9960 apds(colorBus, Config::COLOR_INT_PIN);
Servo scoop;
ESP32PWM motorPwm[4];
QTRSensors qtr;
Mode mode = Mode::STOPPED;
bool motorsReady = false, colorReady = false, lineCalibrated = false;
bool manualArmed = false;
ControllerPtr activeController = nullptr;
bool previousA = false, previousX = false, previousY = false;
uint32_t autonomousStarted = 0, lastTelemetry = 0;
uint32_t lastControllerReport = 0;
int leftCommand = 0, rightCommand = 0;

struct LineState {
    uint16_t values[Config::LINE_COUNT] = {};
    uint16_t calibrationSamples = 0;
    uint32_t lastCalibration = 0, lastTick = 0, lostAt = 0;
    float previousError = 0;
    int lastDirection = 1;
    uint8_t reacquireCount = 0;
    bool searching = false, derivativeReady = false;
} lineState;

// Compare proportions instead of raw RGB counts. Multiplying the illumination
// by a common factor then has much less effect on the learned color.
struct ColorSample {
    float r = 0, g = 0, b = 0;
    int clear = 0;
    bool valid = false;
};

struct ColorState {
    ColorPhase phase = ColorPhase::LEARN_FIRST;
    ColorSample first, second, candidate, latest;
    uint8_t candidateCount = 0, matchCount = 0;
    uint32_t phaseStarted = 0, lastPoll = 0, lastReading = 0;
    uint32_t cornerCount = 0;
} colorState;

// Unsigned subtraction keeps every timer correct across millis() rollover.
bool elapsed(uint32_t now, uint32_t since, uint32_t duration) { return static_cast<uint32_t>(now - since) >= duration; }

int clampMotor(int value) { return constrain(value, -255, 255); }

// Always clear the inactive H-bridge input before energizing the other one.
// PWM objects and the servo use the SAME allocator, avoiding timer collisions.
void writeWheel(uint8_t index, int command) {
    ESP32PWM &forward = motorPwm[index], &reverse = motorPwm[index + 1];
    if (command > 0) {
        reverse.writeScaled(0);
        forward.writeScaled(command / 255.0f);
    } else {
        forward.writeScaled(0);
        reverse.writeScaled(-command / 255.0f);
    }
}

void drive(int left, int right) {
    leftCommand = clampMotor(left);
    rightCommand = clampMotor(right);
    if (!motorsReady) return;
    left = clampMotor(static_cast<int>(leftCommand * Config::LEFT_TRIM));
    right = clampMotor(static_cast<int>(rightCommand * Config::RIGHT_TRIM));
    writeWheel(0, Config::INVERT_LEFT ? -left : left);
    writeWheel(2, Config::INVERT_RIGHT ? -right : right);
}

void stopRobot(const char* reason) {
    drive(0, 0);
    mode = Mode::STOPPED;
    manualArmed = false;
    Console.printf("STOP: %s\n", reason);
}

// direction: -1 = left, +1 = right.
void pivot(int direction, int speed) { drive(direction * speed, -direction * speed); }

int joystickToSpeed(int value) {
    value = constrain(value, -512, 512);
    int magnitude = abs(value);
    if (magnitude <= Config::DEADZONE) return 0;
    int speed = (magnitude - Config::DEADZONE) * Config::MANUAL_MAX / (512 - Config::DEADZONE);
    return value < 0 ? -speed : speed;
}

void runManual(ControllerPtr controller) {
    int throttle = joystickToSpeed(-controller->axisY());
    int steering = joystickToSpeed(controller->axisRX());
    if (!manualArmed) {
        drive(0, 0);
        if (throttle == 0 && steering == 0) manualArmed = true;
        return;
    }
    int left = throttle + steering, right = throttle - steering;
    // Scale BOTH wheels together to preserve steering when the mix saturates.
    int peak = max(abs(left), abs(right));
    if (peak > Config::MANUAL_MAX) {
        left = left * Config::MANUAL_MAX / peak;
        right = right * Config::MANUAL_MAX / peak;
    }
    drive(left, right);
    // Mechanical and driver challenges remain under manual control.
    if (controller->r1()) {
        scoop.write(Config::SCOOP_DOWN);
    } else if (controller->l1()) {
        scoop.write(Config::SCOOP_UP);
    }
}

void beginCalibration(uint32_t now) {
    drive(0, 0);
    mode = Mode::CALIBRATING;
    lineCalibrated = false;
    qtr.resetCalibration();
    lineState.calibrationSamples = 0;
    lineState.lastCalibration = now;
    Console.printf("CALIBRATE: sweep BOTH sensors over black AND white for 5 seconds.\n");
}

void runCalibration(uint32_t now) {
    drive(0, 0);
    if (!elapsed(now, lineState.lastCalibration, Config::CALIBRATION_INTERVAL_MS)) return;
    lineState.lastCalibration = now;
    qtr.calibrate();
    if (++lineState.calibrationSamples < Config::CALIBRATION_SAMPLES) return;

    // No usable contrast means calibrated values would be misleading.
    bool good = qtr.calibrationOn.initialized && qtr.calibrationOn.minimum && qtr.calibrationOn.maximum;
    if (good) {
        for (uint8_t i = 0; i < Config::LINE_COUNT; ++i) {
            int span = static_cast<int>(qtr.calibrationOn.maximum[i]) - qtr.calibrationOn.minimum[i];
            Console.printf("QTR %u raw min=%u max=%u span=%d\n", i, qtr.calibrationOn.minimum[i],
                           qtr.calibrationOn.maximum[i], span);
            if (span < Config::CALIBRATION_MIN_SPAN) good = false;
        }
    }
    lineCalibrated = good;
    stopRobot(good ? "Calibration passed. B=manual, A=line, X=color." : "Calibration failed. Press Y and sweep BOTH sensors over black/white.");
}

bool lineVisible() {
    return lineState.values[0] >= Config::LINE_VISIBLE || lineState.values[1] >= Config::LINE_VISIBLE;
}

void beginLine(uint32_t now) {
    drive(0, 0);
    if (!lineCalibrated) {
        stopRobot("Press Y to calibrate the line sensors first.");
        return;
    }
    lineState = LineState{};
    qtr.readCalibrated(lineState.values);
    if (!lineVisible()) {
        stopRobot("Place the line sensors on the starting line, then press A.");
        return;
    }
    lineState.lastTick = now;
    mode = Mode::LINE;
    autonomousStarted = now;
    Console.printf("LINE: following. B cancels. A retry must start at the beginning.\n");
}

void runLine(uint32_t now) {
    if (!elapsed(now, lineState.lastTick, Config::LINE_INTERVAL_MS)) return;
    lineState.lastTick = now;
    qtr.readCalibrated(lineState.values);
    bool visible = lineVisible();
    if (!visible && !lineState.searching) {
        lineState.searching = true;
        lineState.lostAt = now;
        lineState.reacquireCount = 0;
        lineState.derivativeReady = false;
    }
    if (lineState.searching) {
        if (visible) {
            if (++lineState.reacquireCount < Config::LINE_REACQUIRE_SAMPLES) {
                drive(0, 0);
                return;
            }
            lineState.searching = false;
        } else {
            lineState.reacquireCount = 0;
            uint32_t lost = now - lineState.lostAt;
            if (lost < Config::LINE_GAP_MS) {
                drive(Config::LINE_SLOW_SPEED, Config::LINE_SLOW_SPEED);
            } else {
                uint32_t sweep = lost - Config::LINE_GAP_MS;
                // Scan toward the last-seen line, through the other side, then
                // approximately back to the original heading. Never search forever.
                if (sweep < Config::LINE_SWEEP_MS) {
                    pivot(lineState.lastDirection, Config::LINE_TURN_SPEED);
                } else if (sweep < 3 * Config::LINE_SWEEP_MS) {
                    pivot(-lineState.lastDirection, Config::LINE_TURN_SPEED);
                } else if (sweep < 4 * Config::LINE_SWEEP_MS) {
                    pivot(lineState.lastDirection, Config::LINE_TURN_SPEED);
                } else {
                    stopRobot("Line ended OR was lost. Verify position; retry from the start.");
                }
            }
            return;
        }
    }
    // Weighted two-sensor position: negative means line left, positive right.
    // A positive correction speeds up the left wheel, turning the robot right.
    float left = lineState.values[0], right = lineState.values[1];
    float error = (right - left) / (right + left);
    float derivative = lineState.derivativeReady ? error - lineState.previousError : 0;
    lineState.previousError = error;
    lineState.derivativeReady = true;
    if (fabsf(error) > 0.08f) lineState.lastDirection = error > 0 ? 1 : -1;
    int correction = static_cast<int>(Config::LINE_KP * error + Config::LINE_KD * derivative);
    int base = fabsf(error) > 0.45f ? Config::LINE_SLOW_SPEED : Config::LINE_SPEED;
    drive(base + correction, base - correction);
}

float colorDistanceSquared(const ColorSample& a, const ColorSample& b) {
    float dr = a.r - b.r, dg = a.g - b.g, db = a.b - b.b;
    return dr * dr + dg * dg + db * db;
}

bool colorsMatch(const ColorSample& a, const ColorSample& b) {
    return a.valid && b.valid && colorDistanceSquared(a, b) <= Config::COLOR_MATCH_DISTANCE * Config::COLOR_MATCH_DISTANCE;
}

// Return true only for a NEW reading. A dark/neutral reading is still new but
// has valid=false, so gaps break a streak instead of reusing the last tile.
bool pollColor(uint32_t now, ColorSample& sample) {
    if (!elapsed(now, colorState.lastPoll, Config::COLOR_POLL_MS)) return false;
    colorState.lastPoll = now;
    if (!apds.colorAvailable()) return false;
    int r, g, b, clear;
    if (!apds.readColor(r, g, b, clear)) return false;
    colorState.lastReading = now;
    sample = ColorSample{};
    sample.clear = clear;
    int total = r + g + b;
    if (r < 0 || g < 0 || b < 0 || total <= 0 || clear < Config::COLOR_MIN_CLEAR ||
        max(max(r, g), max(b, clear)) >= Config::COLOR_SATURATION) {
        return true;
    }
    sample.r = static_cast<float>(r) / total;
    sample.g = static_cast<float>(g) / total;
    sample.b = static_cast<float>(b) / total;
    float chroma = max(max(sample.r, sample.g), sample.b) - min(min(sample.r, sample.g), sample.b);
    sample.valid = chroma >= Config::MIN_CHROMA;
    return true;
}

// A candidate must remain consistent for multiple independent sensor frames.
// The average is anchored by the first frame to avoid drifting across an edge.
bool learnColor(const ColorSample& sample, ColorSample& learned) {
    if (!sample.valid) {
        colorState.candidateCount = 0;
        return false;
    }
    if (!colorState.candidateCount || !colorsMatch(sample, colorState.candidate)) {
        colorState.candidate = sample;
        colorState.candidateCount = 1;
        learned = sample;
    } else {
        float n = colorState.candidateCount;
        learned.r = (learned.r * n + sample.r) / (n + 1);
        learned.g = (learned.g * n + sample.g) / (n + 1);
        learned.b = (learned.b * n + sample.b) / (n + 1);
        ++colorState.candidateCount;
    }
    return colorState.candidateCount >= Config::COLOR_LEARN_SAMPLES;
}

bool confirmedColor(const ColorSample& sample, const ColorSample& target) {
    if (!colorsMatch(sample, target)) {
        colorState.matchCount = 0;
        return false;
    }
    if (colorState.matchCount < Config::COLOR_MATCH_SAMPLES) ++colorState.matchCount;
    return colorState.matchCount >= Config::COLOR_MATCH_SAMPLES;
}

void setColorPhase(ColorPhase phase, uint32_t now) {
    drive(0, 0);
    colorState.phase = phase;
    colorState.phaseStarted = now;
    colorState.matchCount = colorState.candidateCount = 0;
    Console.printf("COLOR phase=%u, A corners=%lu\n", static_cast<unsigned>(phase), static_cast<uint32_t>(colorState.cornerCount));
}

void beginColor(uint32_t now) {
    drive(0, 0);
    if (!colorReady) {
        stopRobot("APDS9960 not initialized. Check 3.3V power and I2C wiring; reboot.");
        return;
    }
    colorState = ColorState{};
    colorState.lastReading = colorState.lastPoll = now;
    mode = Mode::COLOR;
    autonomousStarted = now;
    setColorPhase(ColorPhase::LEARN_FIRST, now);
    Console.printf("COLOR: hold still on the first tile (A), pointed at the second tile (B).\n");
}

void runColor(uint32_t now) {
    ColorSample sample;
    bool fresh = pollColor(now, sample);
    if (fresh) colorState.latest = sample;
    if (elapsed(now, colorState.lastReading, Config::COLOR_SENSOR_TIMEOUT_MS)) {
        stopRobot("No fresh APDS9960 readings. Check the sensor and wiring.");
        return;
    }
    uint32_t phaseTime = now - colorState.phaseStarted;
    switch (colorState.phase) {
        case ColorPhase::LEARN_FIRST:
            drive(0, 0);
            // Discard initial frames that may have integrated before X was pressed.
            if (phaseTime < Config::COLOR_SETTLE_MS) return;
            if (fresh && learnColor(sample, colorState.first)) {
                Console.printf("Learned A: %.3f %.3f %.3f\n", colorState.first.r, colorState.first.g, colorState.first.b);
                setColorPhase(ColorPhase::LEARN_SECOND, now);
            } else if (phaseTime >= Config::COLOR_FIRST_TIMEOUT_MS) {
                stopRobot("Cannot learn A: check lighting, saturation, sensor height, and tile color.");
            }
            break;

        case ColorPhase::LEARN_SECOND:
            drive(Config::COLOR_SPEED, Config::COLOR_SPEED);
            if (fresh) {
                // Reject A, neutral floor, and blended boundary readings.
                float separation = colorDistanceSquared(sample, colorState.first);
                if (!sample.valid || separation < Config::COLOR_MIN_SEPARATION * Config::COLOR_MIN_SEPARATION) {
                    colorState.candidateCount = 0;
                } else if (learnColor(sample, colorState.second)) {
                    Console.printf("Learned B: %.3f %.3f %.3f\n", colorState.second.r, colorState.second.g, colorState.second.b);
                    setColorPhase(ColorPhase::DRIVE_TO_FIRST, now);
                }
            }
            if (mode == Mode::COLOR && colorState.phase == ColorPhase::LEARN_SECOND &&
                phaseTime >= Config::COLOR_TRAVEL_TIMEOUT_MS) {
                stopRobot("No distinct second tile found. Check alignment, background, and thresholds.");
            }
            break;

        case ColorPhase::DRIVE_TO_FIRST:
            drive(Config::COLOR_SPEED, Config::COLOR_SPEED);
            if (fresh && confirmedColor(sample, colorState.first)) {
                ++colorState.cornerCount;
                setColorPhase(ColorPhase::ADVANCE_AT_FIRST, now);
            } else if (phaseTime >= Config::COLOR_TRAVEL_TIMEOUT_MS) {
                stopRobot("Expected A was not reached. Check heading, speed, and color matching.");
            }
            break;

        case ColorPhase::ADVANCE_AT_FIRST:
            if (phaseTime >= Config::COLOR_PIVOT_ADVANCE_MS) {
                setColorPhase(ColorPhase::SETTLE_AT_FIRST, now);
            } else {
                drive(Config::COLOR_SPEED, Config::COLOR_SPEED);
            }
            break;

        case ColorPhase::SETTLE_AT_FIRST:
            drive(0, 0);
            if (phaseTime >= Config::COLOR_SETTLE_MS) setColorPhase(ColorPhase::SCAN_LEFT, now);
            break;

        case ColorPhase::SCAN_LEFT:
        case ColorPhase::SCAN_RIGHT:
            // Only search for B during outward sweeps. Return sweeps ignore tiles.
            // Two matches reduce the chance that a mixed edge picks a false heading.
            if (fresh && confirmedColor(sample, colorState.second)) {
                setColorPhase(ColorPhase::SETTLE_ON_SECOND, now);
            } else if (phaseTime >= Config::COLOR_SCAN_HALF_MS) {
                setColorPhase(colorState.phase == ColorPhase::SCAN_LEFT ? ColorPhase::RETURN_FROM_LEFT : ColorPhase::RETURN_FROM_RIGHT, now);
            } else {
                pivot(colorState.phase == ColorPhase::SCAN_LEFT ? -1 : 1, Config::COLOR_TURN_SPEED);
            }
            break;

        case ColorPhase::RETURN_FROM_LEFT:
            if (phaseTime >= Config::COLOR_SCAN_HALF_MS) {
                setColorPhase(ColorPhase::SCAN_RIGHT, now);
            } else {
                pivot(1, Config::COLOR_TURN_SPEED);
            }
            break;

        case ColorPhase::RETURN_FROM_RIGHT:
            if (phaseTime >= Config::COLOR_SCAN_HALF_MS) {
                stopRobot("No next B found after A: final tile OR missed route. Verify before continuing.");
            } else {
                pivot(-1, Config::COLOR_TURN_SPEED);
            }
            break;

        case ColorPhase::SETTLE_ON_SECOND:
            drive(0, 0);
            if (phaseTime < Config::COLOR_SETTLE_MS) break;
            // Require B again while stopped, so a fleeting edge is not accepted.
            if (fresh && confirmedColor(sample, colorState.second)) {
                setColorPhase(ColorPhase::DRIVE_TO_FIRST, now);
            } else if (phaseTime >= Config::COLOR_SETTLE_MS + Config::COLOR_SENSOR_TIMEOUT_MS) {
                stopRobot("B disappeared after turning. Reduce turn speed or fix sensor geometry.");
            }
            break;
    }
}

ControllerPtr findController() {
    // Retain the same controller until it disconnects; do not swap mid-run.
    if (activeController && activeController->isConnected() && activeController->isGamepad()) return activeController;
    for (auto controller : myControllers) {
        if (controller && controller->isConnected() && controller->isGamepad()) return controller;
    }
    return nullptr;
}

void printTelemetry(uint32_t now) {
    if (!Config::TELEMETRY || !elapsed(now, lastTelemetry, Config::TELEMETRY_INTERVAL_MS)) return;
    lastTelemetry = now;
    if (mode == Mode::LINE) {
        Console.printf("QTR L=%u R=%u; drive=%d,%d\n", lineState.values[0], lineState.values[1], leftCommand, rightCommand);
    } else if (mode == Mode::COLOR) {
        Console.printf("RGB/sum %.3f %.3f %.3f clear=%d usable=%d; drive=%d,%d\n",
                       colorState.latest.r,
                       colorState.latest.g,
                       colorState.latest.b,
                       colorState.latest.clear,
                       colorState.latest.valid,
                       leftCommand, rightCommand);
    }
}

void setup() {
    Serial.begin(115200);
    const uint8_t motorPins[] = {Config::LEFT_IN1, Config::LEFT_IN2, Config::RIGHT_IN1, Config::RIGHT_IN2};
    for (uint8_t pin : motorPins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    scoop.setPeriodHertz(50);
    scoop.attach(Config::SERVO_PIN);
    scoop.write(Config::SCOOP_START);
    for (uint8_t i = 0; i < 4; ++i) {
        motorPwm[i].attachPin(motorPins[i], Config::MOTOR_PWM_HZ, 8);
        motorPwm[i].writeScaled(0);
    }
    motorsReady = motorPwm[0].attached() && motorPwm[1].attached() && motorPwm[2].attached() && motorPwm[3].attached();
    drive(0, 0);
    esp_log_level_set("gpio", ESP_LOG_ERROR);
    BP32.setup(&onConnectedController, &onDisconnectedController);
    // Keep existing pairing keys and the allowlist installed by your starter
    // callbacks. Do not erase pairing on every boot.
    uni_bt_allowlist_set_enabled(true);

    colorBus.begin(Config::SDA_PIN, Config::SCL_PIN, 100000);
    colorBus.setTimeOut(20);  // Bound I2C bus waits if a wire comes loose.
    apds.setInterruptPin(Config::COLOR_INT_PIN);
    colorReady = apds.begin();
    Console.printf("APDS9960: %s\n", colorReady ? "ready" : "FAILED (manual/line still available)");

    // Pololu's analog QTR library assumes 10-bit raw ADC values (0..1023).
    // No SharpIR sensors are read, so keep this consistent throughout the run.
    analogReadResolution(10);
    qtr.setTypeAnalog();
    qtr.setSensorPins(Config::LINE_PINS, Config::LINE_COUNT);
    qtr.setSamplesPerSensor(4);
    if (!motorsReady) {
        stopRobot("Motor PWM initialization failed.");
        return;
    }
    beginCalibration(millis());
}

void loop() {
    // Each pass does a small amount of work. There are no blocking motion or
    // search loops: controller updates continue during every autonomous phase.
    BP32.update();
    uint32_t now = millis();
    ControllerPtr controller = findController();
    if (controller != activeController) {
        bool hadController = activeController != nullptr;
        activeController = controller;
        previousA = controller && controller->a();
        previousX = controller && controller->x();
        previousY = controller && controller->y();
        lastControllerReport = now;
        manualArmed = false;
        // Connecting during stationary calibration is fine; disconnecting
        // during a moving mode always cancels that run, even if another pad exists.
        if (mode != Mode::CALIBRATING || hadController) {
            stopRobot(controller ? "Controller changed. Release buttons; B=manual." : "Controller disconnected. Reconnect, then B=manual.");
        }
    }

    if (!motorsReady) {
        drive(0, 0);
        delay(5);
        return;
    }
    // isConnected() can remain true for seconds after the radio goes out of
    // range. Track new reports from THIS controller, not BP32.update() globally.
    if (controller && controller->hasData()) lastControllerReport = now;
    if (controller && elapsed(now, lastControllerReport, Config::CONTROLLER_TIMEOUT_MS)) {
        if (mode == Mode::CALIBRATING) {
            runCalibration(now);
        } else if (mode != Mode::STOPPED) {
            stopRobot("Controller reports timed out. Restore link, then restart.");
        }
        delay(5);
        return;
    }
    if (controller) {
        bool a = controller->a(), x = controller->x(), y = controller->y();
        if (controller->b()) {
            // B outranks every other button and never moves the wheels or scoop.
            if (mode != Mode::MANUAL) Console.printf("MANUAL: center sticks after releasing B.\n");
            mode = Mode::MANUAL;
            manualArmed = false;
            drive(0, 0);
            previousA = a;
            previousX = x;
            previousY = y;
            delay(5);
            return;
        }
        if (y && !previousY) {
            beginCalibration(now);
        } else if (mode != Mode::CALIBRATING && a && !previousA) {
            beginLine(now);
        } else if (mode != Mode::CALIBRATING && x && !previousX) {
            beginColor(now);
        }
        previousA = a;
        previousX = x;
        previousY = y;
    }
    if (mode == Mode::CALIBRATING) {
        runCalibration(now);  // Can calibrate without a controller; motors off.
    } else if (!controller) {
        if (mode != Mode::STOPPED) stopRobot("No controller connected.");
        drive(0, 0);
    } else if ((mode == Mode::LINE || mode == Mode::COLOR) &&
               elapsed(now, autonomousStarted, Config::AUTONOMOUS_TIMEOUT_MS)) {
        stopRobot("Autonomous runtime limit reached.");
    } else {
        switch (mode) {
            case Mode::MANUAL:
                runManual(controller);
                break;
            case Mode::LINE:
                runLine(now);
                break;
            case Mode::COLOR:
                runColor(now);
                break;
            default:
                drive(0, 0);
                break;
        }
    }
    printTelemetry(now);
    delay(5);  // Yield to Bluetooth/RTOS tasks, including with no controller.
}
