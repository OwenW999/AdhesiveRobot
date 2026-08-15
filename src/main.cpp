#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>

// ============================================================
// PINS
// ============================================================

#define STEP_PIN_OUTPUT       50
#define DIR_PIN_OUTPUT        52

#define STEP_PIN_INPUT        26
#define DIR_PIN_INPUT         28

#define STEP_PIN_FIXTURE      22
#define DIR_PIN_FIXTURE       24

#define STEP_PIN_NEMA23_TABLE 32
#define DIR_PIN_NEMA23_TABLE  34

#define STEP_PIN_NEMA23_SCREW 25
#define DIR_PIN_NEMA23_SCREW  27

#define LIMIT_NEMA23          23
#define LIMIT_NEMA23SCREW     47
#define LIMIT_BELT            39

#define SERVO1_PIN            51
#define SERVO2_PIN            53

#define HALL_PIN              43

#define BUTTON_PIN            41
#define BUTTON_PIN_LIGHT      33


// ============================================================
// MOTORS
// ============================================================

AccelStepper motorOutBelt(
    AccelStepper::DRIVER,
    STEP_PIN_OUTPUT,
    DIR_PIN_OUTPUT
);

AccelStepper motorInBelt(
    AccelStepper::DRIVER,
    STEP_PIN_INPUT,
    DIR_PIN_INPUT
);

AccelStepper motorFixturePlayer(
    AccelStepper::DRIVER,
    STEP_PIN_FIXTURE,
    DIR_PIN_FIXTURE
);

AccelStepper motorNema23Table(
    AccelStepper::DRIVER,
    STEP_PIN_NEMA23_TABLE,
    DIR_PIN_NEMA23_TABLE
);

AccelStepper motorNema23Screw(
    AccelStepper::DRIVER,
    STEP_PIN_NEMA23_SCREW,
    DIR_PIN_NEMA23_SCREW
);


// ============================================================
// SERVOS
// ============================================================

Servo servo1;
Servo servo2;


// ============================================================
// CONSTANTS
// ============================================================

const int NUM_FIXTURES = 5;

// Change this after you determine the actual number of
// steps required to rotate the fixture carousel 72 degrees.
const long FIXTURE_72_DEG_STEPS = 1050;

const double SCREW_LEAD_MM = 5.0;
const double SCREW_STEPS_PER_REV = 1600.0;
const double SCREW_STEPS_PER_MM = SCREW_STEPS_PER_REV / SCREW_LEAD_MM;

// Screw positions
// 0 = top
// negative = down

const long SCREW_TOP  = 0;
const long SCREW_DOWN = -3000;


// Table
const long QUARTER_REV_STEPS = 1000;


// ============================================================
// SERVO POSITIONS
// ============================================================

// Open claw
const int SERVO1_OPEN = 20;
const int SERVO2_OPEN = 0;

// Grab mount
const int SERVO1_GRAB = 120;
const int SERVO2_GRAB = 173;

const int SERVO1_GRAB_LOOSE = 115;
const int SERVO2_GRAB_LOOSE = 168;

// ============================================================
// FIXTURE VARIABLES
// ============================================================

// Which fixture is currently underneath the arm.
//
// 0, 1, 2, 3, or 4
int currentFixture = 0;


// Does each fixture currently contain VHB?
//
// Example:
// { true, false, true, false, false }
//
// means fixtures 0 and 2 have VHB.
bool fixtureHasVHB[NUM_FIXTURES] = {
    false,
    false,
    false,
    false,
    false
};


// True while the mount is down on the fixture
// and the fixture is actively being used.
bool armUsingFixture = false;


// ============================================================
// OTHER VARIABLES
// ============================================================

bool mountReady = false;


// ============================================================
// HOMING STATE MACHINE
// ============================================================

enum HomeState {

    HOME_SCREW,

    HOME_SCREW_BACKOFF,

    HOME_TABLE,

    HOME_TABLE_BACKOFF,

    HOME_FIXTURE,

    HOME_COMPLETE
};

HomeState homeState = HOME_SCREW;

bool homed = false;


// ============================================================
// MAIN PROCESS STATE MACHINE
// ============================================================

enum ProcessState {

    PROCESS_WAIT_MOUNT,

    PROCESS_PICK_MOUNT,

    PROCESS_MOVE_TO_FIXTURE,

    PROCESS_WAIT_FOR_VHB,

    PROCESS_LOWER_ONTO_VHB,

    PROCESS_APPLY_VHB,

    PROCESS_RAISE_MOUNT,

    PROCESS_MOVE_TO_OUTPUT,

    PROCESS_RELEASE_MOUNT

};

ProcessState processState = PROCESS_WAIT_MOUNT;


// ============================================================
// FIXTURE STATE MACHINE
// ============================================================

enum FixtureState {

    FIXTURE_IDLE,

    FIXTURE_ROTATING

};

FixtureState fixtureState = FIXTURE_IDLE;

enum MoveSequenceState {
    SEQUENCE_IDLE,
    SEQUENCE_MOVING_TABLE,
    SEQUENCE_MOVING_SCREW,
    SEQUENCE_DONE
};

MoveSequenceState moveSequenceState = SEQUENCE_IDLE;

// ============================================================
// SETUP
// ============================================================

void setup() {

    Serial.begin(9600);


    // --------------------------------------------------------
    // INPUTS
    // --------------------------------------------------------

    pinMode(LIMIT_NEMA23, INPUT_PULLUP);
    pinMode(LIMIT_NEMA23SCREW, INPUT_PULLUP);

    pinMode(LIMIT_BELT, INPUT_PULLUP);

    pinMode(HALL_PIN, INPUT_PULLUP);

    pinMode(BUTTON_PIN, INPUT_PULLUP);


    // --------------------------------------------------------
    // BUTTON LIGHT
    // --------------------------------------------------------

    pinMode(BUTTON_PIN_LIGHT, OUTPUT);

    digitalWrite(BUTTON_PIN_LIGHT, LOW);


    // --------------------------------------------------------
    // MOTOR SETTINGS
    // --------------------------------------------------------

    motorOutBelt.setMaxSpeed(2000);
    motorOutBelt.setSpeed(-1800);


    motorInBelt.setMaxSpeed(2000);
    motorInBelt.setSpeed(-1800);


    motorFixturePlayer.setMaxSpeed(2000);
    motorFixturePlayer.setAcceleration(1000);
    motorFixturePlayer.setSpeed(500);


    motorNema23Table.setMaxSpeed(2000);
    motorNema23Table.setAcceleration(1000);


    motorNema23Screw.setMaxSpeed(2000);
    motorNema23Screw.setAcceleration(1000);


    // --------------------------------------------------------
    // SERVOS
    // --------------------------------------------------------

    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);


    // Start with claw open
    openGripper();


    Serial.println("ROBOT STARTING");
}


// ============================================================
// LOOP
// ============================================================

void loop() {

    // All motors that are currently moving get serviced
    runMotors();

    // --------------------------------------------------------
    // HOMING
    // --------------------------------------------------------
    if (!homed) {

        runHoming();

        updateFixtureButtonLight();

        return;
    }

    // --------------------------------------------------------
    // MAIN ROBOT PROCESS
    // --------------------------------------------------------
    runMainProcess();

    // --------------------------------------------------------
    // FIXTURE CAROUSEL
    // --------------------------------------------------------
    runFixtureManager();

    // --------------------------------------------------------
    // BUTTON LIGHT
    // --------------------------------------------------------
    updateFixtureButtonLight();
}


// ============================================================
// RUN MOTORS
// ============================================================

void runMotors() {
    motorOutBelt.runSpeed();
    motorInBelt.runSpeed();
    motorFixturePlayer.run();
    motorNema23Table.run();
    motorNema23Screw.run();
}


// ============================================================
// HOMING
// ============================================================

void runHoming() {
    switch (homeState) {
        // ====================================================
        // HOME SCREW
        // ====================================================
        case HOME_SCREW:

            Serial.println("Homing screw...");

            // Move toward switch
            motorNema23Screw.setSpeed(1800);

            if (digitalRead(LIMIT_NEMA23SCREW) == LOW) {
                motorNema23Screw.runSpeed();
            }
            else {
                // Switch hit
                motorNema23Screw.stop();
                motorNema23Screw.setCurrentPosition(0);
                Serial.println("Screw homed.");

                // Back away
                motorNema23Screw.moveTo(-2000);
                homeState = HOME_SCREW_BACKOFF;
            }
            break;

        // ====================================================
        // SCREW BACKOFF
        // ====================================================

        case HOME_SCREW_BACKOFF:
            if (motorNema23Screw.distanceToGo() == 0) {
                Serial.println("Screw backoff complete.");
                homeState = HOME_TABLE;
            }
            break;

        // ====================================================
        // HOME TABLE
        // ====================================================

        case HOME_TABLE:
            Serial.println("Homing table...");
            motorNema23Table.setSpeed(-500);

            if (digitalRead(LIMIT_NEMA23) == LOW) {
                motorNema23Table.runSpeed();
            }
            else {
                motorNema23Table.stop();
                motorNema23Table.setCurrentPosition(0);
                Serial.println("Table homed.");

                motorNema23Table.moveTo(QUARTER_REV_STEPS);
                homeState = HOME_TABLE_BACKOFF;
            }

            break;


        // ====================================================
        // TABLE BACKOFF
        // ====================================================

        case HOME_TABLE_BACKOFF:
            if (motorNema23Table.distanceToGo() == 0) {
                Serial.println("Table backoff complete.");
                homeState = HOME_FIXTURE;
            }
            break;

        // ====================================================
        // HOME FIXTURE
        // ====================================================

        case HOME_FIXTURE:
            Serial.println("Homing fixture...");
            motorFixturePlayer.setSpeed(500);

            if (digitalRead(HALL_PIN) == LOW) {
                motorFixturePlayer.runSpeed();
            }
            else {
                motorFixturePlayer.stop();
                motorFixturePlayer.setCurrentPosition(0);
                currentFixture = 0;
                Serial.println("Fixture homed.");
                homeState = HOME_COMPLETE;
            }
            break;

        // ====================================================
        // COMPLETE
        // ====================================================

        case HOME_COMPLETE:
            Serial.println("HOMING COMPLETE");
            homed = true;
            break;
    }
}

// ============================================================
// MAIN PROCESS
// ============================================================

void runMainProcess() {
    switch (processState) {

        // ====================================================
        // WAIT FOR MOUNT
        // ====================================================

        case PROCESS_WAIT_MOUNT:
            /*
             * Input belt brings mount into position.
             */
            if (digitalRead(LIMIT_BELT) == HIGH) {
                Serial.println("MOUNT READY");
                mountReady = true;

                // Stop input belt
                motorInBelt.setSpeed(0);
                processState = PROCESS_PICK_MOUNT;
            }
            break;

        // ====================================================
        // PICK UP MOUNT
        // ====================================================

        case PROCESS_PICK_MOUNT:
            Serial.println("PICKING UP MOUNT");
            /*
             * TODO:
             *
             * Move table/arm to mount
             *
             * Move screw to correct height
             *
             * Grab mount
             */
            tableToDeg(216);
            ScrewToMM(79);
            grabMount();
            releaseGripSlighty();
            ScrewToMM(82);
            grabMount();
            processState = PROCESS_MOVE_TO_FIXTURE;
            break;

        // ====================================================
        // MOVE TO FIXTURE
        // ====================================================

        case PROCESS_MOVE_TO_FIXTURE:
            Serial.println("MOVING TO FIXTURE");

            tableToDeg(113.5);

            if (motorNema23Table.distanceToGo() == 0) {
                processState = PROCESS_WAIT_FOR_VHB;
            }

            break;

        // ====================================================
        // WAIT FOR VHB
        // ====================================================

        case PROCESS_WAIT_FOR_VHB:
            /*
             * DO NOT LOWER THE ARM YET.
             *
             * Wait until the fixture underneath the arm
             * actually contains VHB.
             */

            if (fixtureHasVHB[currentFixture]) {

                Serial.println("VHB FIXTURE READY");

                processState = PROCESS_LOWER_ONTO_VHB;
            }

            break;


        // ====================================================
        // LOWER ONTO VHB
        // ====================================================

        case PROCESS_LOWER_ONTO_VHB:

            /*
             * The mount is still being held by the arm.
             *
             * We are NOT releasing it onto the fixture.
             */

            Serial.println("LOWERING ONTO VHB");
            armUsingFixture = true;
            ScrewToMM(63);

            if (motorNema23Screw.distanceToGo() == 0) {
                processState = PROCESS_APPLY_VHB;
            }
            break;

        // ====================================================
        // APPLY VHB
        // ====================================================

        case PROCESS_APPLY_VHB:

            Serial.println("APPLYING VHB");

            tableToDeg(36);

            processState = PROCESS_RAISE_MOUNT;

            break;


        // ====================================================
        // RAISE MOUNT
        // ====================================================

        case PROCESS_RAISE_MOUNT:

            Serial.println("RAISING MOUNT");


            motorNema23Screw.moveTo(SCREW_TOP);


            if (motorNema23Screw.distanceToGo() == 0) {

                /*
                 * Mount is now clear of fixture.
                 */

                armUsingFixture = false;


                /*
                 * VHB has been consumed from this fixture.
                 */

                fixtureHasVHB[currentFixture] = false;


                processState = PROCESS_MOVE_TO_OUTPUT;
            }

            break;


        // ====================================================
        // MOVE TO OUTPUT
        // ====================================================

        case PROCESS_MOVE_TO_OUTPUT:

            Serial.println("MOVING TO OUTPUT");


            /*
             * TODO:
             *
             * Move table/arm to output position.
             */


            if (motorNema23Table.distanceToGo() == 0) {

                processState = PROCESS_RELEASE_MOUNT;
            }

            break;


        // ====================================================
        // RELEASE MOUNT
        // ====================================================

        case PROCESS_RELEASE_MOUNT:

            Serial.println("RELEASING MOUNT");

            openGripper();


            // Mount is no longer at input
            mountReady = false;


            // Restart input belt
            motorInBelt.setSpeed(-1800);


            processState = PROCESS_WAIT_MOUNT;

            break;
    }
}


// ============================================================
// FIXTURE MANAGER
// ============================================================

void runFixtureManager() {

    // --------------------------------------------------------
    // If arm is currently using fixture, DON'T ROTATE
    // --------------------------------------------------------

    if (armUsingFixture) {

        return;
    }


    // --------------------------------------------------------
    // If fixture is currently rotating
    // --------------------------------------------------------

    if (fixtureState == FIXTURE_ROTATING) {

        if (motorFixturePlayer.distanceToGo() == 0) {

            fixtureState = FIXTURE_IDLE;

            Serial.print("CURRENT FIXTURE: ");
            Serial.println(currentFixture);
        }

        return;
    }


    // --------------------------------------------------------
    // Check button
    // --------------------------------------------------------

    if (digitalRead(BUTTON_PIN) == LOW) {

        if (canRotateFixture()) {

            rotateFixture();
        }
    }
}


// ============================================================
// CAN ROTATE FIXTURE
// ============================================================

bool canRotateFixture() {

    /*
     * Fixture can rotate as long as:
     *
     * 1. Arm isn't currently using fixture
     * 2. Fixture isn't already rotating
     */

    if (armUsingFixture) {
        return false;
    }

    if (fixtureState != FIXTURE_IDLE) {
        return false;
    }

    return true;
}


// ============================================================
// ROTATE FIXTURE
// ============================================================

void rotateFixture() {

    Serial.println("ROTATING FIXTURE");


    /*
     * Rotate exactly one fixture position = 72 degrees.
     */

    motorFixturePlayer.move(FIXTURE_72_DEG_STEPS);


    /*
     * Update which fixture is underneath the arm.
     */

    currentFixture++;

    if (currentFixture >= NUM_FIXTURES) {

        currentFixture = 0;
    }


    fixtureState = FIXTURE_ROTATING;
}


// ============================================================
// BUTTON LIGHT
// ============================================================

void updateFixtureButtonLight() {

    /*
     * Light means:
     *
     * "Pressing the button will rotate the fixture."
     */

    if (homed && canRotateFixture()) {

        digitalWrite(BUTTON_PIN_LIGHT, HIGH);
    }
    else {

        digitalWrite(BUTTON_PIN_LIGHT, LOW);
    }
}


// ============================================================
// GRIPPER
// ============================================================

void openGripper() {

    servo1.write(SERVO1_OPEN);
    servo2.write(SERVO2_OPEN);
}


void grabMount() {

    servo1.write(SERVO1_GRAB);
    servo2.write(SERVO2_GRAB);
}

void releaseGripSlighty() {

    servo1.write(SERVO1_GRAB_LOOSE);
    servo2.write(SERVO2_GRAB_LOOSE);
}

// ============================================================
// TABLE POSITION HELPER
// ============================================================

bool moveTableThenScrew(double degree, double mm) {
    switch (moveSequenceState) {

        case SEQUENCE_IDLE:
            tableToDeg(degree);
            moveSequenceState = SEQUENCE_MOVING_TABLE;
            break;

        case SEQUENCE_MOVING_TABLE:

            if (motorNema23Table.distanceToGo() == 0) {
                ScrewToMM(mm);
                moveSequenceState = SEQUENCE_MOVING_SCREW;
            }
            break;

        case SEQUENCE_MOVING_SCREW:

            if (motorNema23Screw.distanceToGo() == 0) {
                moveSequenceState = SEQUENCE_DONE;
            }

            break;

        case SEQUENCE_DONE:
            moveSequenceState = SEQUENCE_IDLE;
            return true;
    }
    return false;
}

void tableToDeg(double degree) {

    motorNema23Table.moveTo(
        degree / 360.0 * 1600.0 * 2.625
    );
}

void ScrewToMM(double mm) {

    motorNema23Screw.moveTo(
        -(mm * SCREW_STEPS_PER_MM)
    );
}