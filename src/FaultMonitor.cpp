#include "FaultMonitor.h"

FaultMonitor::FaultMonitor(MotorController& motor)
    : motor_(motor) {
}

FaultReport FaultMonitor::currentReport() const {
    FaultReport report;
    report.code = code_;
    report.critical = critical_;
    report.message = message_;
    return report;
}

FaultReport FaultMonitor::latch(FaultCode code, const char* message, bool critical) {
    code_ = code;
    message_ = message;
    critical_ = critical;
    if (critical) {
        latched_ = true;
        motor_.safeStop();
    }
    return currentReport();
}

FaultReport FaultMonitor::evaluate(const MotorSnapshot& snapshot) {
    if (latched_) {
        return currentReport();
    }

    if (snapshot.measuredRpm > MotorLimits::MaxRpm) {
        return latch(
            FaultCode::Overspeed,
            "Measured speed exceeded the maximum RPM.",
            true);
    }

    if (snapshot.encoderUpdateMissing) {
        ++missedTicks_;
        if (missedTicks_ >= MotorLimits::CommunicationTimeoutTicks) {
            return latch(
                FaultCode::CommunicationTimeout,
                "Encoder feedback was not received.",
                true);
        }
    } else {
        missedTicks_ = 0;
    }

    const int error = snapshot.controlErrorRpm;
    const int absoluteError = error < 0 ? -error : error;
    if (!snapshot.encoderUpdateMissing &&
        snapshot.state == MotorState::Running &&
        absoluteError > MotorLimits::ExcessiveErrorRpm) {
        ++excessiveErrorTicks_;
        if (excessiveErrorTicks_ >= MotorLimits::ExcessiveErrorTickLimit) {
            return latch(
                FaultCode::ExcessiveError,
                "Speed error stayed above the allowed limit.",
                true);
        }
    } else if (!snapshot.encoderUpdateMissing) {
        excessiveErrorTicks_ = 0;
    }

    return currentReport();
}

void FaultMonitor::reportInvalidCommand() {
    if (latched_) {
        return;
    }
    code_ = FaultCode::InvalidCommand;
    message_ = "The command is not valid in the current state.";
    critical_ = false;
}

void FaultMonitor::reportSpeedOutOfRange() {
    if (latched_) {
        return;
    }
    code_ = FaultCode::SpeedOutOfRange;
    message_ = "Target speed must be from 0 to 3000 RPM.";
    critical_ = false;
}

void FaultMonitor::clearCommandFault() {
    if (latched_) {
        return;
    }
    code_ = FaultCode::None;
    message_ = "No fault.";
    critical_ = false;
}

void FaultMonitor::clear() {
    code_ = FaultCode::None;
    message_ = "No fault.";
    critical_ = false;
    latched_ = false;
    missedTicks_ = 0;
    excessiveErrorTicks_ = 0;
}

FaultCode FaultMonitor::activeCode() const {
    return code_;
}

const char* FaultMonitor::message() const {
    return message_;
}

bool FaultMonitor::isCritical() const {
    return critical_;
}
