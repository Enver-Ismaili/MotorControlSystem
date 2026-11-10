#pragma once

// Shared simulation types. These describe a software model of a motor,
// not registers or signals on physical hardware.

enum class MotorState {
    Stopped,
    Running,
    Fault
};

enum class FaultCode {
    None,
    InvalidCommand,
    SpeedOutOfRange,
    CommunicationTimeout,
    Overspeed,
    ExcessiveError
};

enum class CommandResult {
    Accepted,
    InvalidCommand,
    SpeedOutOfRange
};

namespace MotorLimits {
    constexpr int MinRpm = 0;
    constexpr int MaxRpm = 3000;

    // Discrete proportional gain applied to the drive command each tick:
    // adjustment = error * KpNumerator / KpDenominator.
    constexpr int KpNumerator = 4;
    constexpr int KpDenominator = 5;

    // Simulated encoder closes this fraction of the gap each tick.
    constexpr int LagNumerator = 7;
    constexpr int LagDenominator = 10;

    constexpr int ExcessiveErrorRpm = 500;
    constexpr int ExcessiveErrorTickLimit = 5;
    constexpr int CommunicationTimeoutTicks = 3;
}

struct MotorSnapshot {
    MotorState state = MotorState::Stopped;
    int targetRpm = 0;
    int driveCommandRpm = 0;
    int measuredRpm = 0;
    int controlErrorRpm = 0;
    bool encoderUpdateMissing = false;
};

inline const char* toString(MotorState state) {
    switch (state) {
    case MotorState::Stopped:
        return "STOPPED";
    case MotorState::Running:
        return "RUNNING";
    case MotorState::Fault:
        return "FAULT";
    }
    return "UNKNOWN";
}

inline const char* toString(FaultCode code) {
    switch (code) {
    case FaultCode::None:
        return "NONE";
    case FaultCode::InvalidCommand:
        return "INVALID_COMMAND";
    case FaultCode::SpeedOutOfRange:
        return "SPEED_OUT_OF_RANGE";
    case FaultCode::CommunicationTimeout:
        return "COMMUNICATION_TIMEOUT";
    case FaultCode::Overspeed:
        return "OVERSPEED";
    case FaultCode::ExcessiveError:
        return "EXCESSIVE_ERROR";
    }
    return "UNKNOWN";
}

inline const char* toString(CommandResult result) {
    switch (result) {
    case CommandResult::Accepted:
        return "Accepted";
    case CommandResult::InvalidCommand:
        return "Invalid command";
    case CommandResult::SpeedOutOfRange:
        return "Speed out of range";
    }
    return "Unknown";
}
