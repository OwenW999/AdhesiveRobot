#include <Arduino.h>
#include <AccelStepper.h>

#define STEP_PIN_OUTPUT 50
#define DIR_PIN_OUTPUT 52
#define STEP_PIN_INPUT 26
#define DIR_PIN_INPUT 28
#define STEP_PIN_FIXTURE 22
#define DIR_PIN_FIXTURE 24

#define STEP_PIN_NEMA23 32
#define DIR_PIN_NEMA23 34

AccelStepper motorOutBelt(AccelStepper::DRIVER, STEP_PIN_OUTPUT, DIR_PIN_OUTPUT);
AccelStepper motorInBelt(AccelStepper::DRIVER, STEP_PIN_INPUT, DIR_PIN_INPUT);
AccelStepper motorFixturePlayer(AccelStepper::DRIVER, STEP_PIN_FIXTURE, DIR_PIN_FIXTURE);
AccelStepper motorNema23(AccelStepper::DRIVER, STEP_PIN_NEMA23, DIR_PIN_NEMA23);

const long QUARTER_REV_STEPS = 1000; // change to match your DM542 microstepping DIP setting

void setup() {
    motorOutBelt.setMaxSpeed(2000);
    motorOutBelt.setSpeed(-1800);
    motorInBelt.setMaxSpeed(2000);
    motorInBelt.setSpeed(-1800);
    motorFixturePlayer.setMaxSpeed(2000);
    motorFixturePlayer.setSpeed(500);

    motorNema23.setMaxSpeed(2000);
    motorNema23.setAcceleration(1000);
    motorNema23.setCurrentPosition(0);
    motorNema23.moveTo(QUARTER_REV_STEPS);
}

void loop() {
    motorOutBelt.runSpeed();
    motorInBelt.runSpeed();
    motorFixturePlayer.runSpeed();

    // Flip target once current target is reached
    if (motorNema23.distanceToGo() == 0) {
        if (motorNema23.currentPosition() == 0) {
            motorNema23.moveTo(QUARTER_REV_STEPS);
        } else {
            motorNema23.moveTo(0);
        }
    }
    motorNema23.run();
}