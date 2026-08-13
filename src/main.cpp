#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>

#define STEP_PIN_OUTPUT 50
#define DIR_PIN_OUTPUT 52
#define STEP_PIN_INPUT 26
#define DIR_PIN_INPUT 28
#define STEP_PIN_FIXTURE 22
#define DIR_PIN_FIXTURE 24

#define STEP_PIN_NEMA23_TABLE 32
#define DIR_PIN_NEMA23_TABLE 34

#define STEP_PIN_NEMA23_SCREW 25
#define DIR_PIN_NEMA23_SCREW 27

#define LIMIT_NEMA23 23
#define LIMIT_NEMA23SCREW 47
#define LIMIT_BELT 39

#define SERVO1_PIN 51
#define SERVO2_PIN 53

Servo servo1;
Servo servo2;

AccelStepper motorOutBelt(AccelStepper::DRIVER, STEP_PIN_OUTPUT, DIR_PIN_OUTPUT);
AccelStepper motorInBelt(AccelStepper::DRIVER, STEP_PIN_INPUT, DIR_PIN_INPUT);
AccelStepper motorFixturePlayer(AccelStepper::DRIVER, STEP_PIN_FIXTURE, DIR_PIN_FIXTURE);
AccelStepper motorNema23Table(AccelStepper::DRIVER, STEP_PIN_NEMA23_TABLE, DIR_PIN_NEMA23_TABLE);
AccelStepper motorNema23Screw(AccelStepper::DRIVER, STEP_PIN_NEMA23_SCREW, DIR_PIN_NEMA23_SCREW);

const long QUARTER_REV_STEPS = 1000;

bool homedTable = false;
bool homedScrew = false;
bool mountReady = false;

// 0 is top for screw!! negative is down!!

void setup() {
    Serial.begin(9600);

    pinMode(LIMIT_NEMA23, INPUT_PULLUP);

    motorOutBelt.setMaxSpeed(2000);
    motorOutBelt.setSpeed(-1800);

    motorInBelt.setMaxSpeed(2000);
    motorInBelt.setSpeed(-1800);

    motorFixturePlayer.setMaxSpeed(2000);
    motorFixturePlayer.setSpeed(500);

    motorNema23Table.setMaxSpeed(2000);
    motorNema23Table.setAcceleration(1000);

    motorNema23Table.setSpeed(-500);

    motorNema23Screw.setMaxSpeed(2000);
    motorNema23Screw.setAcceleration(1000);
    motorNema23Screw.setSpeed(1800);

    // -------------------------
    // SERVOS
    // -------------------------
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);

    // open claw basically is 20 servo1 0 servo2
    servo1.write(20);
    servo2.write(0);

    // delay(5000);

    // // 1/8 revolution on a 270° servo
    // // servo1.write(34);
    // // 120 about where to grab a mount for servo 2

    // servo1.write(50);
    // servo2.write(173);
}

void loop() {
    // -------------------------
    // 1. HOME SCREW FIRST
    // -------------------------
    if (!homedScrew) {

        // NC switch:
        // LOW  = not pressed
        // HIGH = pressed
        if (digitalRead(LIMIT_NEMA23SCREW) == LOW) {

            // Move toward screw limit switch
            motorNema23Screw.runSpeed();

        } else {

            // Switch reached
            motorNema23Screw.stop();

            // This physical location is position 0
            motorNema23Screw.setCurrentPosition(0);

            homedScrew = true;

            // Move away from the limit switch
            motorNema23Screw.moveTo(-2000);
            motorNema23Screw.run();
        }

        return;
    }


    // -------------------------
    // 2. HOME TABLE AFTER SCREW
    // -------------------------
    if (!homedTable) {

        // NC switch:
        // LOW  = not pressed
        // HIGH = pressed
        if (digitalRead(LIMIT_NEMA23) == LOW) {

            // Move toward table limit switch
            motorNema23Table.runSpeed();

        } else {

            // Switch reached
            motorNema23Table.stop();

            // This physical location is now position 0
            motorNema23Table.setCurrentPosition(0);

            // Move to starting position
            motorNema23Table.moveTo(QUARTER_REV_STEPS);

            homedTable = true;
        }

        return;
    }


    // -------------------------
    // 3. NORMAL OPERATION
    // -------------------------


    // Other motors continue running
    motorOutBelt.runSpeed();
    motorFixturePlayer.runSpeed();
    // Serial.println("YOOOOOO");
    if (digitalRead(LIMIT_BELT) == HIGH) {
        mountReady = true;
    }

    if (!mountReady) {
        motorInBelt.runSpeed();
    }
    
    // Table movement
    if (motorNema23Table.distanceToGo() == 0) {

        if (motorNema23Table.currentPosition() == 0) {
            motorNema23Table.moveTo(QUARTER_REV_STEPS);
        }
        else {
            motorNema23Table.moveTo(0);
        }
    }

    motorNema23Table.run();

    // Screw movement
    if (motorNema23Screw.distanceToGo() == 0) {

        if (motorNema23Screw.currentPosition() == -3000) {
            motorNema23Screw.moveTo(-6000);
        }
        else {
            motorNema23Screw.moveTo(-3000);
        }
    }

    motorNema23Screw.run();
}