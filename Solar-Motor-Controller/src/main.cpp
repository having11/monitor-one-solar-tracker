#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"
#include <Wire.h>

constexpr uint16_t MotorSteps = 200;
constexpr uint8_t MotorXRpm = 15;
constexpr uint8_t MotorYRpm = 90;
constexpr uint8_t Microsteps = 32;

constexpr uint8_t MotorXDir = 7;
constexpr uint8_t MotorXStep = 6;
constexpr uint8_t MotorXEn = 8;
constexpr uint8_t MotorYDir = 4;
constexpr uint8_t MotorYStep = 5;
constexpr uint8_t MotorYEn = 12;
constexpr uint8_t HomeYPin = 2;

constexpr uint8_t NumAxes = 2;
constexpr uint8_t I2cAddress = 0x10;

constexpr uint8_t StepperTeeth = 20;
constexpr uint8_t PulleyTeeth = 60;
constexpr double DegPerStepperXRotation = 360.0 / (PulleyTeeth / StepperTeeth);
constexpr double DistanceMMPerStepperYRotation = 1.25;
constexpr double DistanceMMPerDegree = 5;

constexpr double MinTiltDeg = 25;
constexpr double MaxTiltDeg = 40;
constexpr double MinRollDeg = -30;
constexpr double MaxRollDeg = 0;

enum class ControllerCommandType {
    GetAngles = 0,
    GetMinMaxAngles,
    GoToAngles,
    HomeAxes
};

typedef struct {
    ControllerCommandType type;
    union {
        // Tilt, Roll
        double targetAngles[2];
    } data;
} ControllerCommand_t;

static void homeXAxis(void);
static void receiveI2c(int bytesToRead);
static void requestI2c(void);
static void handleI2cCommand(void);
static void moveToAngles(double tilt, double roll);

volatile bool i2cFlag = false;
static ControllerCommand_t lastCommand;
// Tilt, Roll
static double currentAngles[2] = { MinTiltDeg, MaxRollDeg };
static uint8_t buf[sizeof(ControllerCommand_t)];

// Roll stepper
BasicStepperDriver stepperX(MotorSteps, MotorXDir, MotorXStep, MotorXEn);
// Tilt stepper
BasicStepperDriver stepperY(MotorSteps, MotorYDir, MotorYStep, MotorYEn);

SyncDriver controller(stepperX, stepperY);

void setup() {
    Serial.begin(115200);
    pinMode(HomeYPin, INPUT_PULLUP);

    stepperX.begin(MotorXRpm, Microsteps);
    stepperY.begin(MotorYRpm, Microsteps);

    stepperX.setEnableActiveState(LOW);
    stepperY.setEnableActiveState(LOW);

    stepperX.enable();
    stepperY.enable();
    stepperX.stop();

    homeXAxis();

    Wire.onReceive(receiveI2c);
    Wire.onRequest(requestI2c);
    Wire.begin(I2cAddress);
}

void loop() {
    if (i2cFlag) {
        i2cFlag = false;
        handleI2cCommand();
    }
}

void receiveI2c(int bytesToRead) {
    Wire.readBytes((uint8_t*)buf, bytesToRead);

    i2cFlag = true;
}

void requestI2c() {
    switch (lastCommand.type) {
        case ControllerCommandType::GetAngles:
            Wire.write((uint8_t*)currentAngles, sizeof(double));
            Wire.write((uint8_t*)(currentAngles + 1), sizeof(double));
            break;
        case ControllerCommandType::GetMinMaxAngles:
            Wire.write((uint8_t*)&MinTiltDeg, sizeof(double));
            Wire.write((uint8_t*)&MaxTiltDeg, sizeof(double));
            Wire.write((uint8_t*)&MinRollDeg, sizeof(double));
            Wire.write((uint8_t*)&MaxRollDeg, sizeof(double));
            break;
        default:
            Wire.write(-1);
            break;
    }
}

void moveToAngles(double tilt, double roll) {
    tilt = constrain(tilt, MinTiltDeg, MaxTiltDeg);
    roll = constrain(roll, MinRollDeg, MaxRollDeg);

    double tiltDelta = tilt - currentAngles[0];
    double rollDelta = roll - currentAngles[1];

    double tiltRotations = tiltDelta * (DistanceMMPerDegree / DistanceMMPerStepperYRotation);
    double rollRotations = rollDelta / DegPerStepperXRotation;

    controller.rotate(rollRotations, tiltRotations);
    delay(500);

    currentAngles[0] = tilt;
    currentAngles[1] = roll;
}

void handleI2cCommand() {
    memcpy(&lastCommand, buf, sizeof(ControllerCommand_t));
    Serial.print("Command: ");
    Serial.println((uint8_t)lastCommand.type);

    switch (lastCommand.type) {
        case ControllerCommandType::GoToAngles:
            moveToAngles(lastCommand.data.targetAngles[0],
                lastCommand.data.targetAngles[1]);
            break;
        case ControllerCommandType::HomeAxes:
            homeXAxis();
            break;
        default:
            break;
    }
}

void homeXAxis() {
    stepperY.enable();

    while (digitalRead(HomeYPin)) {
        stepperY.move(-1);
        delayMicroseconds(2);
    }

    currentAngles[1] = MinTiltDeg;
}