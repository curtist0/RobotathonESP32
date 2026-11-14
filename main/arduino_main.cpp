// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2
// Simple version: A = line follow, X = color run, B = stop/manual, R1/L1 = scoop.
// Keep your existing controller_callbacks.h and its implementation.
// Uses TWO QTR outputs: GPIO32 = left, GPIO33 = right.
// Both selected sensors must see black when centered on the line.
// Motor 1 is left, motor 2 is right. Motors use on/off control as in your original.
// This basic follower can miss sharp corners; it does not measure a 90-degree turn.

#include <Arduino.h>
#include <Arduino_APDS9960.h>
#include <ESP32Servo.h>
#include <ESP32SharpIR.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <bits/stdc++.h>
#include <uni.h>

#include "controller_callbacks.h"
#include "sdkconfig.h"

#define Motor1Pin1 18
#define Motor1Pin2 5
#define Motor2Pin1 17
#define Motor2Pin2 16

extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];
TwoWire I2C_0(0);
APDS9960 apds(I2C_0, 0);
Servo Scoop;
QTRSensors qtr;
const uint8_t LINE_PINS[] = {32, 33};
uint16_t sensors[2];
ESP32SharpIR FrontSensor(ESP32SharpIR::GP2Y0A21YK0F, 12);
ESP32SharpIR LeftSensor(ESP32SharpIR::GP2Y0A21YK0F, 14);
ESP32SharpIR RightSensor(ESP32SharpIR::GP2Y0A21YK0F, 27);

enum Mode { MANUAL, LINE, COLOR, STOPPED };
Mode mode = MANUAL;
bool previousA = false, previousX = false;
const int LINE_THRESHOLD = 750;     // Calibrated values: 0 = white, 1000 = black.
const uint32_t SEARCH_TIME = 1000;  // Maximum time searching without seeing black.
int lastTurn = 0;                   // -1 = left, 1 = right, 0 = no direction known yet.
uint32_t lastLineSeen = 0;
bool colorReady = false, haveStartColor = false, leftStartColor = false;
int startR = 0, startG = 0, startB = 0;
uint32_t lastColorReading = 0;

// For each wheel: 1 = forward, -1 = reverse, 0 = stop.
void drive(int left, int right) {
    digitalWrite(Motor1Pin1, left > 0);
    digitalWrite(Motor1Pin2, left < 0);
    digitalWrite(Motor2Pin1, right > 0);
    digitalWrite(Motor2Pin2, right < 0);
}

void driveStop() { drive(0, 0); }
void driveForward() { drive(1, 1); }

void moveNoob(Controller* controller) {
    if (controller->axisY() < -50)
        driveForward();
    else if (controller->axisY() > 50)
        drive(-1, -1);
    else if (controller->axisRX() > 50)
        drive(1, -1);
    else if (controller->axisRX() < -50)
        drive(-1, 1);
    else
        driveStop();
}

void line() {
    analogReadResolution(10);  // Match the QTR library's analog reading range.
    qtr.readCalibrated(sensors);
    analogReadResolution(12);  // Restore the usual ESP32 setting for other sensors.

    bool leftBlack = sensors[0] > LINE_THRESHOLD;
    bool rightBlack = sensors[1] > LINE_THRESHOLD;

    if (leftBlack || rightBlack) lastLineSeen = millis();

    if (leftBlack && rightBlack) {
        driveForward();
    } else if (leftBlack) {
        lastTurn = -1;
        drive(-1, 1);  // Turn left.
    } else if (rightBlack) {
        lastTurn = 1;
        drive(1, -1);  // Turn right.
    } else if (static_cast<uint32_t>(millis() - lastLineSeen) < SEARCH_TIME && lastTurn != 0) {
        drive(lastTurn, -lastTurn);  // Keep turning toward the last-seen line.
    } else {
        driveStop();
        mode = STOPPED;  // Press A again to retry, or B for manual control.
    }
}

// The color routine takes one reading per call so B can still interrupt it.
void colorRun() {
    int r, g, b, ambient;
    if (!colorReady || !apds.colorAvailable() || !apds.readColor(r, g, b, ambient)) {
        if (!colorReady || static_cast<uint32_t>(millis() - lastColorReading) >= 1500) {
            driveStop();
            mode = STOPPED;
        }
        return;
    }
    lastColorReading = millis();

    if (!haveStartColor) {
        startR = r;
        startG = g;
        startB = b;
        haveStartColor = true;
    } else {
        bool sameColor = abs(r - startR) < 7 && abs(g - startG) < 7 && abs(b - startB) < 7;
        if (!sameColor) {
            leftStartColor = true;
        } else if (leftStartColor) {
            driveStop();
            mode = STOPPED;
            return;
        }
    }
    driveForward();
}

void setup() {
    pinMode(Motor1Pin1, OUTPUT);
    pinMode(Motor1Pin2, OUTPUT);
    pinMode(Motor2Pin1, OUTPUT);
    pinMode(Motor2Pin2, OUTPUT);
    driveStop();
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    esp_log_level_set("gpio", ESP_LOG_ERROR);
    uni_bt_allowlist_set_enabled(true);

    Scoop.attach(23);
    I2C_0.begin(21, 22, 100000);
    apds.setInterruptPin(0);
    colorReady = apds.begin();
    FrontSensor.setFilterRate(1.0f);
    LeftSensor.setFilterRate(1.0f);
    RightSensor.setFilterRate(1.0f);

    qtr.setTypeAnalog();
    qtr.setSensorPins(LINE_PINS, 2);
    analogReadResolution(10);
    Console.printf("Move BOTH line sensors across black and white for calibration.\n");
    for (int i = 0; i < 250; i++) {
        qtr.calibrate();
        BP32.update();
        delay(20);
    }
    analogReadResolution(12);
}

void loop() {
    vTaskDelay(1);
    BP32.update();
    ControllerPtr ctl = nullptr;
    for (auto controller : myControllers) {
        if (controller && controller->isConnected() && controller->isGamepad()) {
            ctl = controller;
            break;
        }
    }
    if (!ctl) {
        driveStop();
        mode = MANUAL;
        previousA = previousX = false;
        return;
    }

    bool aPressed = ctl->a();
    bool xPressed = ctl->x();
    if (ctl->r1()) Scoop.write(0);
    if (ctl->l1()) Scoop.write(180);

    if (ctl->b()) {
        mode = MANUAL;
        driveStop();  // Holding B stops even when a joystick is pushed.
    } else {
        if (aPressed && !previousA) {
            mode = LINE;
            lastTurn = 0;
            lastLineSeen = millis();
        } else if (xPressed && !previousX) {
            driveStop();
            mode = COLOR;
            haveStartColor = leftStartColor = false;
            lastColorReading = millis();
        }

        if (mode == LINE) {
            line();
        } else if (mode == COLOR) {
            colorRun();
        } else if (mode == MANUAL) {
            moveNoob(ctl);
        } else {
            driveStop();
        }
    }
    previousA = aPressed;
    previousX = xPressed;
    delay(10);
}
