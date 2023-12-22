#pragma once

#include "Particle.h"

constexpr uint8_t MotorControllerAddress = 0x10;

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

typedef struct {
    union {
        double currentAngles[2];
        double minMaxAngles[4];
        int8_t error;
    } data;
} ControllerReceivedData_t;