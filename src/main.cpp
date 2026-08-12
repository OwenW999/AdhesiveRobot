#include <Arduino.h>
#include <AccelStepper.h>

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

AccelStepper motorOutBelt(AccelStepper::DRIVER, STEP_PIN_OUTPUT, DIR_PIN_OUTPUT);
AccelStepper motorInBelt(AccelStepper::DRIVER, STEP_PIN_INPUT, DIR_PIN_INPUT);
AccelStepper motorFixturePlayer(AccelStepper::DRIVER, STEP_PIN_FIXTURE, DIR_PIN_FIXTURE);
AccelStepper motorNema23Table(AccelStepper::DRIVER, STEP_PIN_NEMA23_TABLE, DIR_PIN_NEMA23_TABLE);
AccelStepper motorNema23Screw(AccelStepper::DRIVER, STEP_PIN_NEMA23_SCREW, DIR_PIN_NEMA23_SCREW);

const long QUARTER_REV_STEPS = 1000;

bool homed = false;

void setup() {
    pinMode(LIMIT_NEMA23, INPUT_PULLUP);

    motorOutBelt.setMaxSpeed(2000);
    motorOutBelt.setSpeed(-1800);

    motorInBelt.setMaxSpeed(2000);
    motorInBelt.setSpeed(-1800);

    motorFixturePlayer.setMaxSpeed(2000);
    motorFixturePlayer.setSpeed(500);

    motorNema23Table.setMaxSpeed(2000);
    motorNema23Table.setAcceleration(1000);

    // Move toward the limit switch
    motorNema23Table.setSpeed(-500);

    motorNema23Screw.setMaxSpeed(2000);
    motorNema23Screw.setAcceleration(1000);
    motorNema23Screw.setSpeed(-1800);

    motorNema23Screw.setCurrentPosition(0);
}

void loop() {

    // Other motors continue running
    motorOutBelt.runSpeed();
    motorInBelt.runSpeed();
    motorFixturePlayer.runSpeed();

    // -------------------------
    // HOMING
    // -------------------------
    if (!homed) {

        // NC switch:
        // LOW  = not pressed
        // HIGH = pressed
        if (digitalRead(LIMIT_NEMA23) == LOW) {
            motorNema23Table.runSpeed();
        }
        else {
            // Switch reached
            motorNema23Table.stop();

            // This physical location is now position 0
            motorNema23Table.setCurrentPosition(0);

            // Start moving toward the other position
            motorNema23Table.moveTo(QUARTER_REV_STEPS);

            homed = true;
        }

        return;
    }

    // -------------------------
    // NORMAL OPERATION
    // -------------------------

    if (motorNema23Table.distanceToGo() == 0) {

        if (motorNema23Table.currentPosition() == 0) {
            motorNema23Table.moveTo(QUARTER_REV_STEPS);
        }
        else {
            motorNema23Table.moveTo(0);
        }
    }

    motorNema23Table.run();

    if (motorNema23Screw.distanceToGo() == 0) {

        if (motorNema23Screw.currentPosition() == 0) {
            motorNema23Screw.moveTo(-5000);
        }
        else {
            motorNema23Screw.moveTo(0);
        }
    }

    motorNema23Screw.run();
}