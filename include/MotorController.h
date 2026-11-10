#pragma once

#include "MotorTypes.h"

// Simulated motor command handler.
// start, stop, and target speed are software state changes. They do not
// generate PWM, talk to a drive, or provide real-time behavior.
class MotorController {
public:
    CommandResult start();
    CommandResult stop();
    CommandResult setTargetSpeed(int rpm);

    // Critical-fault response: command 0 RPM and latch the Fault state.
    void safeStop();

    // Leaves Fault and returns to Stopped. The stored target is kept.
    CommandResult clearFault();

    // One proportional step. While Stopped or Fault the drive command stays at 0.
    void updateControl(int measuredRpm);

    MotorState state() const;
    int targetRpm() const;
    int driveCommandRpm() const;
    int controlErrorRpm() const;

private:
    MotorState state_ = MotorState::Stopped;
    int targetRpm_ = 0;
    int driveCommandRpm_ = 0;
    int controlErrorRpm_ = 0;
};
