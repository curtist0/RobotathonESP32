// Robotathon: simple line following, color navigation, and manual driving.
// Use this in the RobotathonESP32 starter project, not plain ESP-IDF.
// Keep the starter project's controller_callbacks files and other files.
// Read START_HERE.txt before running. All times and speeds need tuning!

#include <Arduino.h>
#include <Arduino_APDS9960.h>
#include <Bluepad32.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <math.h>
#include <uni.h>

#include "controller_callbacks.h"
#include "sdkconfig.h"

extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// ---------- CHANGE THESE TO MATCH YOUR ROBOT ----------
// This example uses a DRV8833-style driver: two input pins per motor.
const int LEFT_IN1 = 16, LEFT_IN2 = 17;
const int RIGHT_IN1 = 18, RIGHT_IN2 = 19;
const bool REVERSE_LEFT = false, REVERSE_RIGHT = false;

// Connect FOUR neighboring QTR-8A outputs, left to right as seen by the robot.
// Change this array if your wiring differs. GPIO34/35 are input-only: OK here.
const uint8_t LINE_PINS[] = {32, 33, 34, 35};
const uint8_t LINE_COUNT = sizeof(LINE_PINS) / sizeof(LINE_PINS[0]);
const int LINE_SPEED = 100, LINE_MAX_SPEED = 180;
const float LINE_KP = 0.08f;   // Larger = stronger steering corrections.
const int LINE_VISIBLE = 200;  // Calibrated readings range from 0 to 1000.
const uint32_t LINE_MISSING_MS = 250;

const int MANUAL_SPEED = 180;   // PWM speed: 0 to 255.
const int STICK_DEADZONE = 45;  // Ignore small joystick movements.
const uint32_t CONTROLLER_TIMEOUT_MS = 1000;

const int COLOR_SPEED = 75, TURN_SPEED = 85;
const uint32_t TURN_90_MS = 450;  // Measure how long YOUR robot takes to turn.
const uint32_t CENTER_MS = 200;   // Move axle toward tile center before turning.
const uint32_t PROBE_MS = 650;    // Enough travel to see an adjacent tile.
const uint32_t STRAIGHT_TIMEOUT_MS = 8000;
const float COLOR_TOLERANCE = 0.16f;
const int MIN_COLOR_SUM = 30;          // Reject readings with too little light.
const float MIN_COLOR_SPREAD = 0.12f;  // Ignore neutral floor/gaps; see guide.
const int STABLE_READINGS = 4;         // Require several matching sensor readings.

// ---------- SENSOR AND CONTROLLER OBJECTS ----------
TwoWire colorWire(0);
APDS9960 colorSensor(colorWire, -1);  // Polling RGB: the interrupt pin is unused.
QTRSensors qtr;
uint16_t lineValues[LINE_COUNT];
ControllerPtr controller = nullptr;
bool receivedPacket = false, cancelled = false;
bool colorReady = false, lineCalibrated = false;
uint32_t lastPacketTime = 0;

// Store proportions instead of exact RGB brightness, which changes with light.
struct Color {
    float r = 0, g = 0, b = 0;
    bool valid = false;
};
Color firstColor, secondColor;

// ---------- BASIC MOVEMENT ----------
void motor(int in1, int in2, int speed) {
    speed = constrain(speed, -255, 255);
    // Switch the inactive side off before powering the other direction.
    if (speed >= 0) {
        analogWrite(in2, 0);
        analogWrite(in1, speed);
    } else {
        analogWrite(in1, 0);
        analogWrite(in2, -speed);
    }
}

void drive(int left, int right) {
    motor(LEFT_IN1, LEFT_IN2, REVERSE_LEFT ? -left : left);
    motor(RIGHT_IN1, RIGHT_IN2, REVERSE_RIGHT ? -right : right);
}

void stopRobot() { drive(0, 0); }

// Refresh Bluetooth often, including inside autonomous movement loops.
bool updateController() {
    bool fresh = BP32.update();
    ControllerPtr previous = controller;
    controller = nullptr;
    for (auto pad : myControllers) {
        if (pad && pad->isConnected() && pad->isGamepad()) {
            controller = pad;
            break;  // Only one controller operates this robot.
        }
    }
    if (controller != previous) receivedPacket = false;
    if (fresh && controller && controller->hasData()) {
        lastPacketTime = millis();
        receivedPacket = true;
    }
    return controller && receivedPacket &&
           millis() - lastPacketTime < CONTROLLER_TIMEOUT_MS;
}

void cancelTask(const char* reason) {
    stopRobot();
    cancelled = true;
    Console.printf("%s\n", reason);
}

bool keepRunning() {
    bool connected = updateController();
    if (!connected || controller->b()) {
        cancelTask("Stopped: B pressed or controller connection lost.");
    }
    return !cancelled;
}

// Unlike a long delay(), this keeps checking the stop button and connection.
bool moveFor(int left, int right, uint32_t duration) {
    uint32_t start = millis();
    while (millis() - start < duration) {
        if (!keepRunning()) return false;
        drive(left, right);
        delay(5);
    }
    stopRobot();
    return true;
}

// ---------- LINE CHALLENGE ----------
void calibrateLine() {
    stopRobot();
    lineCalibrated = false;
    qtr.resetCalibration();
    Console.printf("For 5 seconds, move every line sensor over tape AND floor.\n");
    for (int i = 0; i < 250; ++i) {
        if (!keepRunning()) return;
        qtr.calibrate();
        delay(20);  // Motors stay off while you move the robot.
    }
    if (!qtr.calibrationOn.minimum || !qtr.calibrationOn.maximum) {
        Console.printf("Calibration unavailable. Check the QTR library/setup.\n");
        return;
    }
    for (uint8_t i = 0; i < LINE_COUNT; ++i) {
        int span = qtr.calibrationOn.maximum[i] - qtr.calibrationOn.minimum[i];
        Console.printf("Sensor %u calibration range: %d\n", i, span);
        if (span < 80) {
            Console.printf("Too little contrast. Check height/wiring and calibrate again.\n");
            return;
        }
    }
    lineCalibrated = true;
    Console.printf("Line calibrated. Put robot on the START of the line; press X.\n");
}

bool lineVisible() {
    for (uint8_t i = 0; i < LINE_COUNT; ++i) {
        if (lineValues[i] >= LINE_VISIBLE) return true;
    }
    return false;
}

void followLine() {
    if (!lineCalibrated) {
        Console.printf("Press Y to calibrate the line sensors first.\n");
        return;
    }
    qtr.readLineBlack(lineValues);
    if (!lineVisible()) {
        Console.printf("No starting line visible. Put sensors over the line.\n");
        return;
    }
    uint32_t lastSeen = millis();
    while (keepRunning()) {
        int position = qtr.readLineBlack(lineValues);
        if (lineVisible()) {
            lastSeen = millis();
            int center = (LINE_COUNT - 1) * 1000 / 2;
            int correction = (int)((position - center) * LINE_KP);
            // Line on right -> speed up left wheel and slow down right wheel.
            drive(constrain(LINE_SPEED + correction, 0, LINE_MAX_SPEED),
                  constrain(LINE_SPEED - correction, 0, LINE_MAX_SPEED));
        } else {
            // Briefly creep straight across a tiny gap, then stop if still missing.
            drive(45, 45);
            if (millis() - lastSeen >= LINE_MISSING_MS) {
                Console.printf("Line ended OR line lost. Stopping. If lost, reset at start.\n");
                break;
            }
        }
        delay(5);
    }
    stopRobot();
}

// ---------- COLOR CHALLENGE ----------
float colorDifference(const Color& a, const Color& b) {
    return fabsf(a.r - b.r) + fabsf(a.g - b.g) + fabsf(a.b - b.b);
}

bool readColor(Color& result) {
    result.valid = false;
    uint32_t start = millis();
    while (!colorSensor.colorAvailable()) {
        if (!keepRunning()) return false;
        if (millis() - start > 500) {
            cancelTask("Color sensor timed out. Check sensor wiring.");
            return false;
        }
        delay(5);
    }
    int r, g, b, clear;
    if (!colorSensor.readColor(r, g, b, clear)) {
        cancelTask("Could not read color sensor.");
        return false;
    }
    int total = r + g + b;
    if (r < 0 || g < 0 || b < 0 || total < MIN_COLOR_SUM) return true;
    result.r = (float)r / total;
    result.g = (float)g / total;
    result.b = (float)b / total;
    float spread = fmaxf(result.r, fmaxf(result.g, result.b)) -
                   fminf(result.r, fminf(result.g, result.b));
    result.valid = spread >= MIN_COLOR_SPREAD;
    return true;
}

// Wait for a stable match. This reads colors; it does not choose motor speeds.
// wanted == nullptr means learn a color, excluding the first if already saved.
bool waitForColor(Color& found, const Color* wanted, uint32_t timeout) {
    uint32_t start = millis();
    Color candidate;
    int stable = 0;
    while (millis() - start < timeout && keepRunning()) {
        Color now;
        if (!readColor(now)) return false;
        bool matches = now.valid;
        if (wanted) {
            matches = matches && colorDifference(now, *wanted) < COLOR_TOLERANCE;
        } else if (firstColor.valid) {
            // Reject ambiguous mixtures close to the first color when learning B.
            matches = matches && colorDifference(now, firstColor) > 2 * COLOR_TOLERANCE;
        }
        if (!matches) {
            stable = 0;
        } else {
            if (stable == 0 || colorDifference(now, candidate) >= COLOR_TOLERANCE) {
                candidate = now;
                stable = 1;
            } else {
                ++stable;
            }
            if (stable >= STABLE_READINGS) {
                found = candidate;
                return true;
            }
        }
        delay(5);
    }
    return false;
}

bool driveToColor(const Color* wanted, Color& found, uint32_t timeout) {
    if (!keepRunning()) return false;
    drive(COLOR_SPEED, COLOR_SPEED);
    bool matched = waitForColor(found, wanted, timeout);
    stopRobot();
    return matched;
}

bool probeForSecondColor() {
    // After turning, move far enough for the sensor to reach the next tile.
    uint32_t start = millis();
    Color found;
    bool matched = driveToColor(&secondColor, found, PROBE_MS);
    uint32_t travelTime = millis() - start;
    if (!matched && !cancelled) {
        // Wrong direction: back up to approximately the same turning location.
        moveFor(-COLOR_SPEED, -COLOR_SPEED, travelTime);
    }
    return matched;
}

bool findNextDirection() {
    // Try LEFT, then RIGHT. Never choose the B tile we just came from behind us.
    if (!moveFor(-TURN_SPEED, TURN_SPEED, TURN_90_MS)) return false;
    if (probeForSecondColor()) return true;
    if (cancelled) return false;
    if (!moveFor(TURN_SPEED, -TURN_SPEED, 2 * TURN_90_MS)) return false;
    if (probeForSecondColor()) return true;
    if (cancelled) return false;
    // No continuation found. Restore approximately the original heading.
    moveFor(-TURN_SPEED, TURN_SPEED, TURN_90_MS);
    return false;
}

void followColors() {
    stopRobot();
    if (!colorReady) {
        Console.printf("Color sensor did not initialize. Check wiring, then reboot.\n");
        return;
    }
    firstColor = Color{};
    secondColor = Color{};
    Console.printf("Learning FIRST color. Start on tile 1 facing tile 2.\n");
    if (!waitForColor(firstColor, nullptr, 2000)) {
        if (!cancelled) Console.printf("No stable colored tile found. Check lighting.\n");
        return;
    }
    Console.printf("First RGB proportions: %.2f %.2f %.2f\n",
                   firstColor.r, firstColor.g, firstColor.b);
    if (!driveToColor(nullptr, secondColor, STRAIGHT_TIMEOUT_MS)) {
        if (!cancelled) Console.printf("Second color not found. Stopped.\n");
        return;
    }
    Console.printf("Second RGB proportions: %.2f %.2f %.2f\n",
                   secondColor.r, secondColor.g, secondColor.b);
    while (keepRunning()) {
        Color found;
        // We are on B now, so the next A is a NEW first-color marker.
        if (!driveToColor(&firstColor, found, STRAIGHT_TIMEOUT_MS)) {
            if (!cancelled) Console.printf("Next first-color tile not found. Stopped.\n");
            break;
        }
        if (!moveFor(COLOR_SPEED, COLOR_SPEED, CENTER_MS)) break;
        if (!findNextDirection()) {
            if (!cancelled) Console.printf("No left/right continuation: final A tile OR failed search.\n");
            break;
        }
        // Successful search ends on B; repeat until we reach the next A marker.
    }
    stopRobot();
}

// ---------- MANUAL DRIVING AND BUTTONS ----------
int stickSpeed(int raw) {
    if (abs(raw) < STICK_DEADZONE) return 0;
    return raw * MANUAL_SPEED / 512;
}

void setup() {
    Serial.begin(115200);
    pinMode(LEFT_IN1, OUTPUT);
    pinMode(LEFT_IN2, OUTPUT);
    pinMode(RIGHT_IN1, OUTPUT);
    pinMode(RIGHT_IN2, OUTPUT);
    stopRobot();
    analogReadResolution(10);  // QTR analog library expects 0..1023.
    qtr.setTypeAnalog();
    qtr.setSensorPins(LINE_PINS, LINE_COUNT);
    colorWire.begin(21, 22, 100000);
    colorWire.setTimeOut(50);  // Limit a stalled I2C transaction.
    colorReady = colorSensor.begin();
    if (!colorReady) Console.printf("APDS-9960 not found; color mode unavailable.\n");
    BP32.setup(&onConnectedController, &onDisconnectedController);
    // Use the starter's allowlist_add command to allow YOUR controller first.
    uni_bt_allowlist_set_enabled(true);
    Console.printf("A: color | X: line | Y: calibrate | B: stop | hold L1 + stick: drive\n");
}

void loop() {
    static bool buttonsReleased = false;
    if (!updateController()) {
        stopRobot();
        buttonsReleased = false;
    } else if (controller->b()) {
        stopRobot();
        buttonsReleased = false;
    } else if (buttonsReleased && (controller->a() || controller->x() || controller->y())) {
        stopRobot();
        cancelled = false;
        buttonsReleased = false;  // Holding a button must not restart a task.
        if (controller->y())
            calibrateLine();
        else if (controller->x())
            followLine();
        else
            followColors();
        stopRobot();  // Completion returns to stopped manual mode.
    } else {
        bool faceButtonDown = controller->a() || controller->x() || controller->y();
        if (!faceButtonDown) buttonsReleased = true;
        if (controller->l1() && !faceButtonDown && buttonsReleased) {
            int forward = -stickSpeed(controller->axisY());
            int turn = stickSpeed(controller->axisX());
            // Scale both together if the mix exceeds the selected maximum speed.
            int left = forward + turn, right = forward - turn;
            int peak = max(abs(left), abs(right));
            if (peak > MANUAL_SPEED) {
                left = left * MANUAL_SPEED / peak;
                right = right * MANUAL_SPEED / peak;
            }
            drive(left, right);
        } else
            stopRobot();
    }
    delay(5);  // Yield CPU time to Bluetooth/background work.
}
