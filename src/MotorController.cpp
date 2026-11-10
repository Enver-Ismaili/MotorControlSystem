#include "MotorController.h"

CommandResult MotorController::start() {
    if (state_ != MotorState::Stopped) {
        return CommandResult::InvalidCommand;
    }

    state_ = MotorState::Running;
    return CommandResult::Accepted;
}

CommandResult MotorController::stop() {
    if (state_ != MotorState::Running) {
        return CommandResult::InvalidCommand;
    }

    state_ = MotorState::Stopped;
    driveCommandRpm_ = 0;
    return CommandResult::Accepted;
}

CommandResult MotorController::setTargetSpeed(int rpm) {
    if (state_ == MotorState::Fault) {
        return CommandResult::InvalidCommand;
    }

    if (rpm < MotorLimits::MinRpm || rpm > MotorLimits::MaxRpm) {
        return CommandResult::SpeedOutOfRange;
    }

    targetRpm_ = rpm;
    return CommandResult::Accepted;
}

void MotorController::safeStop() {
    state_ = MotorState::Fault;
    driveCommandRpm_ = 0;
}

CommandResult MotorController::clearFault() {
    if (state_ != MotorState::Fault) {
        return CommandResult::InvalidCommand;
    }

    state_ = MotorState::Stopped;
    driveCommandRpm_ = 0;
    return CommandResult::Accepted;
}

void MotorController::updateControl(int measuredRpm) {
    controlErrorRpm_ = targetRpm_ - measuredRpm;
    if (state_ != MotorState::Running) {
        driveCommandRpm_ = 0;
        return;
    }

    // Velocity-form proportional step: nudge the drive command by Kp * error.
    const int adjustment =
        (controlErrorRpm_ * MotorLimits::KpNumerator) / MotorLimits::KpDenominator;
    int nextCommand = driveCommandRpm_ + adjustment;
    if (nextCommand < MotorLimits::MinRpm) {
        nextCommand = MotorLimits::MinRpm;
    }
    if (nextCommand > MotorLimits::MaxRpm) {
        nextCommand = MotorLimits::MaxRpm;
    }
    driveCommandRpm_ = nextCommand;
}

MotorState MotorController::state() const {
    return state_;
}

int MotorController::targetRpm() const {
    return targetRpm_;
}

int MotorController::driveCommandRpm() const {
    return driveCommandRpm_;
}

int MotorController::controlErrorRpm() const {
    return controlErrorRpm_;
}
