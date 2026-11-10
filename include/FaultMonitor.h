#pragma once

#include "MotorController.h"

struct FaultReport {
    FaultCode code = FaultCode::None;
    bool critical = false;
    const char* message = "No fault.";
};

// Watches simulated feedback and commands. A critical fault calls
// MotorController::safeStop. This is not a hardware fault pin or drive STO.
class FaultMonitor {
public:
    explicit FaultMonitor(MotorController& motor);

    FaultReport evaluate(const MotorSnapshot& snapshot);
    void reportInvalidCommand();
    void reportSpeedOutOfRange();
    void clearCommandFault();
    void clear();

    FaultCode activeCode() const;
    const char* message() const;
    bool isCritical() const;

private:
    FaultReport latch(FaultCode code, const char* message, bool critical);
    FaultReport currentReport() const;

    MotorController& motor_;
    FaultCode code_ = FaultCode::None;
    const char* message_ = "No fault.";
    bool critical_ = false;
    bool latched_ = false;
    int missedTicks_ = 0;
    int excessiveErrorTicks_ = 0;
};
