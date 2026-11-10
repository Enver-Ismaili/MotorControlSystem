#include "EncoderSimulator.h"

#include "MotorTypes.h"

void EncoderSimulator::setStalled(bool stalled) {
    stalled_ = stalled;
}

bool EncoderSimulator::stalled() const {
    return stalled_;
}

int EncoderSimulator::rippleOffset() {
    // Repeating, deterministic offsets. No random generator, so the same
    // command sequence always produces the same feedback.
    static constexpr int kRippleRpm[] = {-18, -6, 0, -12};
    const int offset = kRippleRpm[tick_ % 4];
    ++tick_;
    return offset;
}

int EncoderSimulator::update(int driveCommandRpm) {
    if (!stalled_) {
        const int gap = driveCommandRpm - lagRpm_;
        lagRpm_ += (gap * MotorLimits::LagNumerator) / MotorLimits::LagDenominator;
        if (lagRpm_ < MotorLimits::MinRpm) {
            lagRpm_ = MotorLimits::MinRpm;
        }
    }

    int reported = lagRpm_ + rippleOffset();
    if (reported < MotorLimits::MinRpm) {
        reported = MotorLimits::MinRpm;
    }

    // Keep ordinary ripple inside the legal speed range. Overspeed is
    // reported only when feedback is actually above MaxRpm.
    if (reported > MotorLimits::MaxRpm) {
        reported = MotorLimits::MaxRpm;
    }

    reportedRpm_ = reported;
    return reportedRpm_;
}

int EncoderSimulator::measuredRpm() const {
    return reportedRpm_;
}
