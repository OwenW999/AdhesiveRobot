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
#define HALL_PIN 43
#define BUTTON_PIN 41
#define BUTTON_PIN_LIGHT 33

Servo servo1;
Servo servo2;

AccelStepper motorOutBelt(AccelStepper::DRIVER, STEP_PIN_OUTPUT, DIR_PIN_OUTPUT);
AccelStepper motorInBelt(AccelStepper::DRIVER, STEP_PIN_INPUT, DIR_PIN_INPUT);
AccelStepper motorFixturePlayer(AccelStepper::DRIVER, STEP_PIN_FIXTURE, DIR_PIN_FIXTURE);
AccelStepper motorNema23Table(AccelStepper::DRIVER, STEP_PIN_NEMA23_TABLE, DIR_PIN_NEMA23_TABLE);
AccelStepper motorNema23Screw(AccelStepper::DRIVER, STEP_PIN_NEMA23_SCREW, DIR_PIN_NEMA23_SCREW);

const int NUM_FIXTURES = 5;
const long FIXTURE_72_DEG_STEPS = -1360;

const double SCREW_LEAD_MM = 5.0;
const double SCREW_STEPS_PER_REV = 1600.0;
const double SCREW_STEPS_PER_MM = SCREW_STEPS_PER_REV / SCREW_LEAD_MM;

const long SCREW_TOP = 0;
const long SCREW_DOWN = -3000;
const long QUARTER_REV_STEPS = 1000;


// TUNE MORE
const int SERVO1_OPEN = 0;
const int SERVO2_OPEN = 16;
const int SERVO1_GRAB = 64;
const int SERVO2_GRAB = 269;

const int SERVO1_GRAB_LOOSE = 55;
const int SERVO2_GRAB_LOOSE = 260;

const int SERVO_MIN_US = 500;    // 0°
const int SERVO_MAX_US = 2500;   // 270°
const int SERVO_RANGE_DEG = 270;

void writeDegrees(Servo &servo, float degrees) {
    degrees = constrain(degrees, 0, SERVO_RANGE_DEG);
    int us = map(degrees, 0, SERVO_RANGE_DEG, SERVO_MIN_US, SERVO_MAX_US);
    servo.writeMicroseconds(us);
}

const unsigned long SERVO_MOVE_MS = 900;  // tune to how long your servos actually take
unsigned long servoActionStartTime = 0;

int currentFixture = 0;

bool fixtureHasVHB[NUM_FIXTURES] = {
    false,
    false,
    false,
    false,
    false
};

bool armUsingFixture = false;
bool mountReady = false;

// ============================================================
// HOMING
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
// MAIN PROCESS
// ============================================================

enum ProcessState {
    PROCESS_WAIT_MOUNT,
    PROCESS_PICK_MOUNT,
    PROCESS_MOVE_TO_FIXTURE,
    PROCESS_WAIT_FOR_VHB,
    PROCESS_LOWER_ONTO_VHB,
    PROCESS_RAISE_AWAY_VHB,
    PROCESS_APPLY_VHB,
    PROCESS_RAISE_MOUNT,
    PROCESS_MOVE_TO_OUTPUT,
    PROCESS_RELEASE_MOUNT
};

ProcessState processState = PROCESS_WAIT_MOUNT;

// ============================================================
// FIXTURE
// ============================================================

enum FixtureState {
    FIXTURE_IDLE,
    FIXTURE_ROTATING
};

FixtureState fixtureState = FIXTURE_IDLE;

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void runMotors();
void runHoming();
void runMainProcess();
void runFixtureManager();
void updateFixtureButtonLight();

void openGripper();
void grabMount();
void releaseGripSlighty();

void TableToDeg(double degree);
void ScrewToMM(double mm);

bool canRotateFixture();
void rotateFixture();

// ============================================================
// ACTION SEQUENCE SYSTEM
// ============================================================

enum ActionType {
    ACTION_TABLE,
    ACTION_SCREW,
    ACTION_TABLE_AND_SCREW,
    ACTION_GRAB,
    ACTION_LOOSEN,
    ACTION_OPEN
};

struct Action {
    ActionType type;
    double tableDeg;
    double screwMM;
};

const int MAX_ACTIONS = 20;
Action actions[MAX_ACTIONS];

int actionCount = 0;
int currentAction = 0;
bool actionsRunning = false;
bool actionStarted = false;

void clearActions() {
    actionCount = 0;
    currentAction = 0;
    actionsRunning = false;
    actionStarted = false;
}

void addTable(double degree) {
    actions[actionCount++] = {ACTION_TABLE, degree, 0};
}

void addScrew(double mm) {
    actions[actionCount++] = {ACTION_SCREW, 0, mm};
}

void addTableAndScrew(double degree, double mm) {
    actions[actionCount++] = {ACTION_TABLE_AND_SCREW, degree, mm};
}

void addGrab() {
    actions[actionCount++] = {ACTION_GRAB, 0, 0};
}

void addLoosen() {
    actions[actionCount++] = {ACTION_LOOSEN, 0, 0};
}

void addOpen() {
    actions[actionCount++] = {ACTION_OPEN, 0, 0};
}

bool runActions() {
    if (currentAction >= actionCount) {
        actionsRunning = false;
        return true;
    }

    Action &action = actions[currentAction];

    if (!actionStarted) {
        switch (action.type) {
            case ACTION_TABLE:
                TableToDeg(action.tableDeg);
                break;

            case ACTION_SCREW:
                ScrewToMM(action.screwMM);
                break;

            case ACTION_TABLE_AND_SCREW:
                TableToDeg(action.tableDeg);
                ScrewToMM(action.screwMM);
                break;

            case ACTION_GRAB:
                grabMount();
                servoActionStartTime = millis();
                break;

            case ACTION_LOOSEN:
                releaseGripSlighty();
                servoActionStartTime = millis();
                break;

            case ACTION_OPEN:
                openGripper();
                servoActionStartTime = millis();
                break;
        }

        actionStarted = true;
    }

    if (action.type == ACTION_TABLE &&
        motorNema23Table.distanceToGo() == 0) {
        currentAction++;
        actionStarted = false;
    }

    else if (action.type == ACTION_SCREW &&
             motorNema23Screw.distanceToGo() == 0) {
        currentAction++;
        actionStarted = false;
    }

    else if (action.type == ACTION_TABLE_AND_SCREW &&
             motorNema23Table.distanceToGo() == 0 &&
             motorNema23Screw.distanceToGo() == 0) {
        currentAction++;
        actionStarted = false;
    }

    else if ((action.type == ACTION_GRAB ||
            action.type == ACTION_LOOSEN ||
            action.type == ACTION_OPEN) &&
            (millis() - servoActionStartTime) >= SERVO_MOVE_MS) {
        currentAction++;
        actionStarted = false;
    }

    return false;
}

// output belt managing

bool outputBeltActive = false;
unsigned long outputBeltStopTime = 0;
const unsigned long OUTPUT_CLEAR_MS = 3200;  // tune to your belt/mount

void startOutputBelt() {
    outputBeltActive = true;
    outputBeltStopTime = millis() + OUTPUT_CLEAR_MS;
}

void serviceOutputBelt() {
    if (outputBeltActive) {
        motorOutBelt.runSpeed();

        if ((long)(millis() - outputBeltStopTime) >= 0) {
            outputBeltActive = false;
        }
    }
}

void serviceInputBelt() {
    if (!mountReady && digitalRead(LIMIT_BELT) == HIGH) {
        Serial.println("MOUNT READY");
        mountReady = true;
        motorInBelt.setSpeed(0);
    }
}

// ============================================================
// SETUP
// ============================================================

void setup() {



    Serial.begin(9600);

    pinMode(LIMIT_NEMA23, INPUT_PULLUP);
    pinMode(LIMIT_NEMA23SCREW, INPUT_PULLUP);
    pinMode(LIMIT_BELT, INPUT_PULLUP);
    pinMode(HALL_PIN, INPUT_PULLUP);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    pinMode(BUTTON_PIN_LIGHT, OUTPUT);
    digitalWrite(BUTTON_PIN_LIGHT, LOW);

    motorOutBelt.setMaxSpeed(2000);
    motorOutBelt.setSpeed(-1600);

    motorInBelt.setMaxSpeed(2000);
    motorInBelt.setSpeed(-1800);

    motorFixturePlayer.setMaxSpeed(2000);
    motorFixturePlayer.setAcceleration(1000);
    motorFixturePlayer.setSpeed(-500);

    motorNema23Table.setMaxSpeed(15000);
    motorNema23Table.setAcceleration(5000);

    motorNema23Screw.setMaxSpeed(20000);
    motorNema23Screw.setAcceleration(12000);

    servo1.attach(SERVO1_PIN, SERVO_MIN_US, SERVO_MAX_US);
    servo2.attach(SERVO2_PIN, SERVO_MIN_US, SERVO_MAX_US);

    openGripper();
    // grabMount();

    Serial.println("ROBOT STARTING");
}

// ============================================================
// LOOP
// ============================================================

void loop() {
    // openGripper();
    // delay(5000);
    // grabMount();
    // delay(5000);
    runMotors();

    serviceInputBelt(); 
    if (!homed) {
        runHoming();
        updateFixtureButtonLight();
        return;
    }
    serviceOutputBelt(); 
    

    runMainProcess();
    runFixtureManager();
    updateFixtureButtonLight();
    // TableToDeg(115.2);
    // ScrewToMM(46);
}

// ============================================================
// RUN MOTORS
// ============================================================

void runMotors() {
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
        case HOME_SCREW:
            motorNema23Screw.setSpeed(1800);

            if (digitalRead(LIMIT_NEMA23SCREW) == LOW) {
                motorNema23Screw.runSpeed();
            }
            else {
                motorNema23Screw.stop();
                motorNema23Screw.setCurrentPosition(0);
                Serial.println("Screw homed.");
                
                motorNema23Screw.moveTo(-2000);
                homeState = HOME_SCREW_BACKOFF;
            }
            break;

        case HOME_SCREW_BACKOFF:
            if (motorNema23Screw.distanceToGo() == 0) {
                Serial.println("Screw backoff complete.");
                homeState = HOME_TABLE;
            }
            break;

        case HOME_TABLE:
            motorNema23Table.setSpeed(-300);

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

        case HOME_TABLE_BACKOFF:
            if (motorNema23Table.distanceToGo() == 0) {
                Serial.println("Table backoff complete.");
                homeState = HOME_FIXTURE;
            }
            break;

        case HOME_FIXTURE:
            motorFixturePlayer.setSpeed(-500);

            if (digitalRead(HALL_PIN) == HIGH) {
                motorFixturePlayer.runSpeed();
            }
            else {
                motorFixturePlayer.stop();
                motorFixturePlayer.setCurrentPosition(0);

                motorFixturePlayer.move(-10);  // <-- one-time offset
                currentFixture = 0;
                Serial.println("Fixture homed.");
                homeState = HOME_COMPLETE;
            }
            break;

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
        case PROCESS_WAIT_MOUNT:

            if (!actionsRunning) {
                clearActions();
                addTableAndScrew(216, 43);
                actionsRunning = true;
            }
            if (runActions() && mountReady) {
                processState = PROCESS_PICK_MOUNT;
                Serial.println("PICKING UP MOUNT");
            }
            break;

        case PROCESS_PICK_MOUNT:
            if (!actionsRunning) {
                clearActions();

                addScrew(79);
                addGrab();
                addLoosen();
                addScrew(85);
                addGrab();
                actionsRunning = true;
            }

            if (runActions()) {
                processState = PROCESS_MOVE_TO_FIXTURE;
                Serial.println("MOVING TO FIXTURE");
            }
            break;

        case PROCESS_MOVE_TO_FIXTURE:
            if (!actionsRunning) {

                clearActions();

                addScrew(46);
                addTable(114.65);

                actionsRunning = true;
            }

            if (runActions()) {
                mountReady = false;
                motorInBelt.setSpeed(-1800);
                processState = PROCESS_WAIT_FOR_VHB;
            }
            break;

        case PROCESS_WAIT_FOR_VHB:
            if (fixtureHasVHB[currentFixture] && fixtureState != FIXTURE_ROTATING) {
                Serial.println("VHB FIXTURE READY");
                processState = PROCESS_LOWER_ONTO_VHB;
            }
            break;

        case PROCESS_LOWER_ONTO_VHB:
            // Serial.println("LOWERING ONTO VHB");

            armUsingFixture = true;
            ScrewToMM(61.5);

            if (motorNema23Screw.distanceToGo() == 0) {
                processState = PROCESS_RAISE_AWAY_VHB;
            }
            break;
        
        case PROCESS_RAISE_AWAY_VHB:
            // Serial.println("RAISING FROM VHB");
            
            ScrewToMM(43);

            if (motorNema23Screw.distanceToGo() == 0) {
                armUsingFixture = false;
                fixtureHasVHB[currentFixture] = false;
                processState = PROCESS_APPLY_VHB;
            }
            break;

        case PROCESS_APPLY_VHB:
            if (!actionsRunning) {
                // Serial.println("Rolling");

                clearActions();

                addTableAndScrew(36, 51);
                addScrew(88);
                addTable(54);
                addScrew(70);
                addTable(36);
                addScrew(88);
                addTable(18);

                actionsRunning = true;
            }

            if (runActions()) {
                processState = PROCESS_RAISE_MOUNT;
            }
            break;

        case PROCESS_RAISE_MOUNT:
            // Serial.println("RAISING MOUNT");

            ScrewToMM(38);

            if (motorNema23Screw.distanceToGo() == 0) {
                processState = PROCESS_MOVE_TO_OUTPUT;
            }
            break;

        case PROCESS_MOVE_TO_OUTPUT:
            // Serial.println("MOVING TO OUTPUT");

            if (!actionsRunning) {
                clearActions();

                addTable(280);
                addScrew(85);

                actionsRunning = true;
            }

            if (runActions()) {
                processState = PROCESS_RELEASE_MOUNT;
            }
            break;

        case PROCESS_RELEASE_MOUNT:
            // Serial.println("RELEASING MOUNT");

            // NEED TO RAISE BACK UP BEFORE I START THE PROCESS AGAIN!!!
            if (!actionsRunning) {
                clearActions();

                openGripper();
                addScrew(45);
                
                actionsRunning = true;
            }

            if (runActions()) {
                startOutputBelt(); 
                processState = PROCESS_WAIT_MOUNT;
            }

            break;
    }
}

// ============================================================
// FIXTURE MANAGER
// ============================================================

void runFixtureManager() {
    if (armUsingFixture) {
        return;
    }

    if (fixtureState == FIXTURE_ROTATING) {
        if (motorFixturePlayer.distanceToGo() == 0) {
            fixtureState = FIXTURE_IDLE;

            Serial.print("CURRENT FIXTURE: ");
            Serial.println(currentFixture);
        }

        return;
    }

    if (digitalRead(BUTTON_PIN) == LOW) {
        if (canRotateFixture()) {
            fixtureHasVHB[(currentFixture + 3) % 5] = true;
            rotateFixture();
        }
    }
}

bool canRotateFixture() {
    if (armUsingFixture) {
        return false;
    }

    if (fixtureState != FIXTURE_IDLE) {
        return false;
    }

    return true;
}

void rotateFixture() {
    Serial.println("ROTATING FIXTURE");

    motorFixturePlayer.move(FIXTURE_72_DEG_STEPS);

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
    writeDegrees(servo1, SERVO1_OPEN);
    writeDegrees(servo2, SERVO2_OPEN);
}

void grabMount() {
    writeDegrees(servo1, SERVO1_GRAB);
    writeDegrees(servo2, SERVO2_GRAB);
}

void releaseGripSlighty() {
    writeDegrees(servo1, SERVO1_GRAB_LOOSE);
    writeDegrees(servo2, SERVO2_GRAB_LOOSE);
}

// ============================================================
// TABLE / SCREW POSITION HELPERS
// ============================================================

void TableToDeg(double degree) {
    motorNema23Table.moveTo(
        degree / 360.0 * 1600.0 * 2.625
    );
}

void ScrewToMM(double mm) {
    motorNema23Screw.moveTo(
        -(mm * SCREW_STEPS_PER_MM)
    );
}