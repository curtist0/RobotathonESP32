// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include <Arduino.h>
#include <Arduino_APDS9960.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <ESP32SharpIR.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <bits/stdc++.h>
#include <uni.h>

#include "controller_callbacks.h"
#include "sdkconfig.h"

#define APDS9960_INT 0
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 100000

#define Motor1Pin1 18  // Control pin 1
#define Motor1Pin2 5   // Control pin 2
#define Motor2Pin1 17  // Control pin 3
#define Motor2Pin2 16  // Control pin 4

extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];  // BP32 library allows for up to 4 concurrent controller connections, but we only need 1

TwoWire I2C_0 = TwoWire(0);
APDS9960 apds = APDS9960(I2C_0, APDS9960_INT);
Servo Scoop;
QTRSensors qtr;
uint16_t sensors[2];

ESP32SharpIR FrontSensor(ESP32SharpIR::GP2Y0A21YK0F, 12);
ESP32SharpIR LeftSensor(ESP32SharpIR::GP2Y0A21YK0F, 14);
ESP32SharpIR RightSensor(ESP32SharpIR::GP2Y0A21YK0F, 27);

String ogColor;
bool color = false;
bool GO = false;

void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "DPAD: %d A: %d B: %d X: %d Y: %d LX: %d LY: %d RX: %d RY: %d L1: %d R1: %d L2: %d R2: %d\n",
        ctl->dpad(),  // D-pad
        ctl->a(),     // Letter buttons
        ctl->b(),
        ctl->x(),
        ctl->y(),
        ctl->axisX(),   // (-511 - 512) left X Axis
        ctl->axisY(),   // (-511 - 512) left Y axis
        ctl->axisRX(),  // (-511 - 512) right X axis
        ctl->axisRY(),  // (-511 - 512) right Y axis
        ctl->l1(),      // Bumpers
        ctl->r1(),
        ctl->l2(),
        ctl->r2());
}

void lineSensor() {
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){32, 33}, 2);
    for (uint8_t i = 0; i < 250; i++) {
        Console.printf("calibrating %d/250\n", i);
        qtr.calibrate();
        delay(20);
    }
}

void setup() {
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    esp_log_level_set("gpio", ESP_LOG_ERROR);  // Suppress info log spam from gpio_isr_service
    uni_bt_allowlist_set_enabled(true);
    // Serial.begin(115200);
    pinMode(Motor1Pin1, OUTPUT);
    pinMode(Motor1Pin2, OUTPUT);
    pinMode(Motor2Pin1, OUTPUT);
    pinMode(Motor2Pin2, OUTPUT);

    Scoop.attach(23);
    I2C_0.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    apds.setInterruptPin(APDS9960_INT);
    apds.begin();
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){32, 33}, 2);  // pin numbers go in the curly brackets {}, and number of sensors in use goes after

    for (uint8_t i = 0; i < 250; i++) {
        Console.printf("calibrating %d/250\n", i);
        qtr.calibrate();
        delay(20);
    }
    lineSensor();
    FrontSensor.setFilterRate(1.0f);
    LeftSensor.setFilterRate(1.0f);
    RightSensor.setFilterRate(1.0f);
}

void scoopServo(Controller* controller) {
    if (controller && controller->isConnected()) {
        if (controller->r1()) {
            Scoop.write(0);
        }
        if (controller->l1()) {
            Scoop.write(180);
        }
    }
}

void moveNoob(Controller* controller) {
    Console.printf("CONTROLLER IS MOVING");
    if (controller && controller->isConnected()) {
        if (controller->axisRX() > 0) {
            digitalWrite(Motor1Pin1, HIGH);
            digitalWrite(Motor1Pin2, LOW);
            digitalWrite(Motor2Pin1, LOW);
            digitalWrite(Motor2Pin2, HIGH);
        }
        if (controller->axisRX() < 0) {
            digitalWrite(Motor1Pin1, LOW);
            digitalWrite(Motor1Pin2, HIGH);
            digitalWrite(Motor2Pin1, HIGH);
            digitalWrite(Motor2Pin2, LOW);
        }
        if (controller->axisY() > 0) {
            digitalWrite(Motor1Pin1, LOW);
            digitalWrite(Motor1Pin2, HIGH);
            digitalWrite(Motor2Pin1, LOW);
            digitalWrite(Motor2Pin2, HIGH);
        }
        if (controller->axisY() < 0) {
            digitalWrite(Motor1Pin1, HIGH);
            digitalWrite(Motor1Pin2, LOW);
            digitalWrite(Motor2Pin1, HIGH);
            digitalWrite(Motor2Pin2, LOW);
        }

        if (controller->axisRX() == 0 && controller->axisY() == 0) {
            digitalWrite(Motor1Pin1, LOW);
            digitalWrite(Motor2Pin1, LOW);
            digitalWrite(Motor1Pin2, LOW);
            digitalWrite(Motor2Pin2, LOW);
        }
    }
}

int firstColor = -1;     // -1 = not set yet
bool leftFirst = false;  // true once we've seen a different color
bool autoRun = false;    // true while running autonomously

void driveStop() {
    digitalWrite(Motor1Pin1, LOW);
    digitalWrite(Motor1Pin2, LOW);
    digitalWrite(Motor2Pin1, LOW);
    digitalWrite(Motor2Pin2, LOW);
}

void driveForward() {
    digitalWrite(Motor1Pin1, HIGH);
    digitalWrite(Motor1Pin2, LOW);
    digitalWrite(Motor2Pin1, HIGH);
    digitalWrite(Motor2Pin2, LOW);
}

// monkey see monkey follow line
void monkey(ControllerPtr gp) {
    if (gp->y()) {
        return;
    }

    qtr.readLineBlack(sensors);  // Get calibrated sensor values returned in the sensors array
    int leftSensor = sensors[0];
    int rightSensor = sensors[1];
    int threshold = 500;  // fix later

    if (leftSensor < threshold && rightSensor < threshold) {
        // Move forward
        digitalWrite(Motor1Pin1, LOW);
        digitalWrite(Motor1Pin2, HIGH);
        digitalWrite(Motor2Pin1, LOW);
        digitalWrite(Motor2Pin2, HIGH);
    } else if (leftSensor < threshold && rightSensor >= threshold) {
        // Turn right
        digitalWrite(Motor1Pin1, LOW);
        digitalWrite(Motor1Pin2, HIGH);
        digitalWrite(Motor2Pin1, HIGH);
        digitalWrite(Motor2Pin2, LOW);
    } else if (leftSensor >= threshold && rightSensor < threshold) {
        // Turn left
        digitalWrite(Motor1Pin1, HIGH);
        digitalWrite(Motor1Pin2, LOW);
        digitalWrite(Motor2Pin1, LOW);
        digitalWrite(Motor2Pin2, HIGH);
    } else {
        // Stop
        digitalWrite(Motor1Pin1, LOW);
        digitalWrite(Motor1Pin2, LOW);
        digitalWrite(Motor2Pin1, LOW);
        digitalWrite(Motor2Pin2, LOW);
    }

    Serial.print(sensors[0]);
    Serial.print(" ");
    Serial.println(sensors[1]);

    delay(250);
}

String colorRun(ControllerPtr gp) {
    int r, g, b, a;

    // break statement
    if (gp->y()) {
        return "";
    }
    // Wait until color is read from the sensor
    while (!apds.colorAvailable()) {
        delay(5);
    }
    apds.readColor(r, g, b, a);

    String currentColor = "";
    if (r > g && r > b) {
        currentColor = "Red";
    } else if (g > r && g > b) {
        currentColor = "Green";
    } else if (b > r && b > g) {
        currentColor = "Blue";
    }

    if (color == false) {
        ogColor = currentColor;
        color = true;
    }
    return currentColor;
}

void line(Controller* controller) {
    qtr.readLineBlack(sensors);
    int sensorL = sensors[0];
    int sensorR = sensors[1];
    int thres = 750;

    Console.printf("S1: %d S2: %d\n", sensors[0], sensors[1]);
    delay(250);
    if (sensorL > thres && sensorR > thres) {
        digitalWrite(Motor1Pin1, HIGH);
        digitalWrite(Motor1Pin2, LOW);
        digitalWrite(Motor2Pin1, HIGH);
        digitalWrite(Motor2Pin2, LOW);
        Console.printf("forward");
    } else if (sensorL <= thres && sensorR > thres) {
        digitalWrite(Motor1Pin1, HIGH);
        digitalWrite(Motor1Pin2, LOW);
        digitalWrite(Motor2Pin1, LOW);
        digitalWrite(Motor2Pin2, HIGH);
        Console.printf("right");
    } else if (sensorL > thres && sensorR <= thres) {
        digitalWrite(Motor1Pin1, LOW);
        digitalWrite(Motor1Pin2, HIGH);
        digitalWrite(Motor2Pin1, HIGH);
        digitalWrite(Motor2Pin2, LOW);
        Console.printf("left");
    } else {
        digitalWrite(Motor1Pin1, LOW);
        digitalWrite(Motor2Pin1, LOW);
        digitalWrite(Motor1Pin2, LOW);
        digitalWrite(Motor2Pin2, LOW);
        Console.printf("stop ");
    }
    delay(250);
}

void loop() {
    vTaskDelay(1);
    BP32.update();
    delay(20);
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            moveNoob(myController);
            if (myController->a()) {
                if (!GO) {
                    colorRun(myController);
                    GO = true;
                }
                while (color == true) {
                    BP32.update();
                    // drive forward
                    digitalWrite(Motor1Pin1, HIGH);
                    digitalWrite(Motor1Pin2, LOW);
                    digitalWrite(Motor2Pin1, LOW);
                    digitalWrite(Motor2Pin2, HIGH);
                    // delay checking for color
                    delay(1000);
                    // check for color
                    if (ogColor != colorRun(myController)) {
                        driveStop();
                        // reset flag to allow read
                        color = false;
                        GO = false;
                        break;
                    }
                }
            }
            if (myController->b()) {
                monkey(myController);
            }
        }
    }
}