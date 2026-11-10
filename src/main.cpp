#include "EncoderSimulator.h"
#include "FaultMonitor.h"
#include "MotorController.h"

#include <cstddef>
#include <iostream>
#include <sstream>
#include <string>

namespace {

constexpr int MaxTicksPerCommand = 1000;

struct Simulation {
    MotorController motor;
    EncoderSimulator encoder;
    FaultMonitor faults;

    Simulation() : faults(motor) {}
};

struct Check {
    bool ok = true;
    std::string detail;

    void expect(bool condition, const char* failure) {
        if (ok && !condition) {
            ok = false;
            detail = failure;
        }
    }
};

bool parseInt(const std::string& text, int& value) {
    if (text.empty()) {
        return false;
    }

    std::size_t index = 0;
    int sign = 1;
    if (text[0] == '+' || text[0] == '-') {
        if (text.size() == 1) {
            return false;
        }
        if (text[0] == '-') {
            sign = -1;
        }
        index = 1;
    }

    int result = 0;
    for (; index < text.size(); ++index) {
        const char ch = text[index];
        if (ch < '0' || ch > '9') {
            return false;
        }
        const int digit = ch - '0';
        if (result > (2147483647 - digit) / 10) {
            return false;
        }
        result = result * 10 + digit;
    }

    value = result * sign;
    return true;
}

void applyResult(Simulation& sim, CommandResult result) {
    if (result == CommandResult::Accepted) {
        sim.faults.clearCommandFault();
        return;
    }
    if (result == CommandResult::SpeedOutOfRange) {
        sim.faults.reportSpeedOutOfRange();
        return;
    }
    sim.faults.reportInvalidCommand();
}

// One software step: simulated encoder, proportional update, then fault check.
// This is not a hardware timer period.
void runTicks(Simulation& sim, int count, bool skipEncoder) {
    for (int i = 0; i < count; ++i) {
        MotorSnapshot snapshot;
        snapshot.state = sim.motor.state();
        snapshot.targetRpm = sim.motor.targetRpm();
        snapshot.driveCommandRpm = sim.motor.driveCommandRpm();
        snapshot.encoderUpdateMissing = skipEncoder;

        if (skipEncoder) {
            snapshot.measuredRpm = sim.encoder.measuredRpm();
            snapshot.controlErrorRpm = sim.motor.controlErrorRpm();
        } else {
            const int measured = sim.encoder.update(sim.motor.driveCommandRpm());
            sim.motor.updateControl(measured);
            snapshot.state = sim.motor.state();
            snapshot.driveCommandRpm = sim.motor.driveCommandRpm();
            snapshot.measuredRpm = measured;
            snapshot.controlErrorRpm = sim.motor.controlErrorRpm();
        }

        sim.faults.evaluate(snapshot);
    }
}

void printStatus(const Simulation& sim) {
    // Actual RPM is the simulated encoder reading. There is no second sensor.
    std::cout << "Status: " << toString(sim.motor.state()) << '\n'
              << "Target RPM: " << sim.motor.targetRpm() << '\n'
              << "Actual RPM: " << sim.encoder.measuredRpm() << '\n'
              << "Encoder feedback RPM: " << sim.encoder.measuredRpm() << '\n'
              << "Drive command RPM: " << sim.motor.driveCommandRpm() << '\n'
              << "Control error RPM: " << sim.motor.controlErrorRpm() << '\n'
              << "Fault: " << toString(sim.faults.activeCode()) << '\n'
              << "Message: " << sim.faults.message() << '\n';
}

void printCommandOutcome(const Simulation& sim, CommandResult result) {
    if (result == CommandResult::Accepted) {
        std::cout << "Command accepted.\n";
    } else {
        std::cout << sim.faults.message() << '\n';
    }
    printStatus(sim);
}

void printHelp() {
    std::cout
        << "Commands:\n"
        << "  start          Start the simulated motor\n"
        << "  stop           Normal stop from RUNNING\n"
        << "  speed <rpm>    Set target speed, from 0 to 3000 RPM\n"
        << "  status         Show state, speeds, error, and fault\n"
        << "  tick [n]       Run n software control steps (default 1, max 1000)\n"
        << "  timeout        Skip 3 encoder updates to simulate a communication loss\n"
        << "  clear          Clear a latched fault and return to STOPPED\n"
        << "  test           Run the repeatable scenarios\n"
        << "  help           Show this list\n"
        << "  quit           Exit\n"
        << "Each tick is one software step. It is not a real-time control period.\n";
}

void reportScenario(const char* name, const Check& check, int& failures) {
    if (check.ok) {
        std::cout << "[PASS] " << name << '\n';
        return;
    }
    std::cout << "[FAIL] " << name << " - " << check.detail << '\n';
    ++failures;
}

void scenarioStartStop(int& failures) {
    Simulation sim;
    Check check;
    check.expect(sim.motor.start() == CommandResult::Accepted, "start was rejected");
    check.expect(sim.motor.state() == MotorState::Running, "start did not enter RUNNING");
    check.expect(sim.motor.stop() == CommandResult::Accepted, "stop was rejected");
    check.expect(sim.motor.state() == MotorState::Stopped, "stop did not enter STOPPED");
    check.expect(sim.motor.driveCommandRpm() == 0, "stop left a non-zero drive command");
    reportScenario("Start and stop", check, failures);
}

void scenarioValidSpeeds(int& failures) {
    Simulation sim;
    Check check;
    check.expect(sim.motor.setTargetSpeed(500) == CommandResult::Accepted, "500 RPM was rejected");
    check.expect(sim.motor.targetRpm() == 500, "target was not 500");
    check.expect(sim.motor.setTargetSpeed(1500) == CommandResult::Accepted, "1500 RPM was rejected");
    check.expect(sim.motor.targetRpm() == 1500, "target was not 1500");
    check.expect(sim.motor.setTargetSpeed(3000) == CommandResult::Accepted, "3000 RPM was rejected");
    check.expect(sim.motor.setTargetSpeed(0) == CommandResult::Accepted, "0 RPM was rejected");
    check.expect(sim.motor.targetRpm() == 0, "target was not 0");
    check.expect(sim.motor.setTargetSpeed(1500) == CommandResult::Accepted, "final target was rejected");
    check.expect(sim.motor.start() == CommandResult::Accepted, "start was rejected");
    runTicks(sim, 6, false);
    check.expect(sim.motor.state() == MotorState::Running, "motor left RUNNING");
    check.expect(sim.motor.targetRpm() == 1500, "running target changed");
    check.expect(sim.motor.driveCommandRpm() > 0, "drive command stayed at 0");
    check.expect(sim.encoder.measuredRpm() > 0, "encoder feedback stayed at 0");
    check.expect(sim.faults.activeCode() == FaultCode::None, "a fault was raised for a valid speed");
    reportScenario("Valid target speeds", check, failures);
}

void scenarioRejectInvalid(int& failures) {
    Simulation sim;
    Check check;
    applyResult(sim, sim.motor.setTargetSpeed(1000));
    check.expect(sim.motor.targetRpm() == 1000, "initial target was not stored");

    const CommandResult negative = sim.motor.setTargetSpeed(-1);
    applyResult(sim, negative);
    check.expect(negative == CommandResult::SpeedOutOfRange, "negative speed was not rejected");
    check.expect(sim.motor.targetRpm() == 1000, "negative speed changed the target");
    check.expect(sim.faults.activeCode() == FaultCode::SpeedOutOfRange, "speed fault was not recorded");

    const CommandResult tooHigh = sim.motor.setTargetSpeed(MotorLimits::MaxRpm + 1);
    applyResult(sim, tooHigh);
    check.expect(tooHigh == CommandResult::SpeedOutOfRange, "high speed was not rejected");
    check.expect(sim.motor.targetRpm() == 1000, "high speed changed the target");

    const CommandResult stopped = sim.motor.stop();
    applyResult(sim, stopped);
    check.expect(stopped == CommandResult::InvalidCommand, "stop while STOPPED was accepted");
    check.expect(sim.motor.state() == MotorState::Stopped, "invalid stop changed the state");
    check.expect(sim.faults.activeCode() == FaultCode::InvalidCommand, "invalid stop was not recorded");

    applyResult(sim, sim.motor.start());
    const CommandResult secondStart = sim.motor.start();
    applyResult(sim, secondStart);
    check.expect(secondStart == CommandResult::InvalidCommand, "second start was accepted");
    check.expect(sim.motor.state() == MotorState::Running, "second start changed the state");
    check.expect(sim.faults.activeCode() == FaultCode::InvalidCommand, "second start was not recorded");
    reportScenario("Reject invalid commands", check, failures);
}

void scenarioEncoderAndError(int& failures) {
    Simulation sim;
    Check check;
    sim.motor.setTargetSpeed(1200);
    sim.motor.start();
    runTicks(sim, 1, false);
    const int earlyError = sim.motor.controlErrorRpm();
    const int earlyAbs = earlyError < 0 ? -earlyError : earlyError;

    runTicks(sim, 9, false);
    const int lateError = sim.motor.controlErrorRpm();
    const int lateAbs = lateError < 0 ? -lateError : lateError;

    check.expect(sim.encoder.measuredRpm() != sim.motor.targetRpm() || lateAbs > 0,
                 "encoder matched the target on every sample");
    check.expect(earlyAbs > lateAbs, "control error did not shrink");
    check.expect(lateAbs < 80, "speed did not settle near the target");
    check.expect(sim.motor.state() == MotorState::Running, "tracking raised a critical fault");
    check.expect(sim.faults.activeCode() == FaultCode::None, "tracking recorded a fault");
    reportScenario("Encoder feedback and shrinking error", check, failures);
}

void scenarioExcessiveError(int& failures) {
    Simulation sim;
    Check check;
    sim.motor.setTargetSpeed(1500);
    sim.motor.start();
    // Hold the simulated feedback so the speed error cannot close.
    sim.encoder.setStalled(true);
    runTicks(sim, MotorLimits::ExcessiveErrorTickLimit, false);
    check.expect(sim.faults.activeCode() == FaultCode::ExcessiveError, "excessive error was not detected");
    check.expect(sim.faults.isCritical(), "excessive error was not critical");
    check.expect(sim.motor.state() == MotorState::Fault, "motor did not enter FAULT");
    check.expect(sim.motor.driveCommandRpm() == 0, "safe stop left a non-zero drive command");
    reportScenario("Excessive speed error safe stop", check, failures);
}

void scenarioTimeout(int& failures) {
    Simulation sim;
    Check check;
    sim.motor.setTargetSpeed(800);
    sim.motor.start();
    runTicks(sim, MotorLimits::CommunicationTimeoutTicks - 1, true);
    check.expect(sim.motor.state() == MotorState::Running, "motor faulted before the timeout limit");
    check.expect(sim.faults.activeCode() != FaultCode::CommunicationTimeout,
                 "timeout fault was raised too early");

    runTicks(sim, 1, true);
    check.expect(sim.faults.activeCode() == FaultCode::CommunicationTimeout, "timeout was not detected");
    check.expect(sim.motor.state() == MotorState::Fault, "timeout did not enter FAULT");
    check.expect(sim.motor.driveCommandRpm() == 0, "timeout did not clear the drive command");
    reportScenario("Communication timeout safe stop", check, failures);
}

void scenarioOverspeed(int& failures) {
    Simulation sim;
    Check check;
    sim.motor.setTargetSpeed(1000);
    sim.motor.start();

    // The encoder model clamps normal ripple to the legal range, so this
    // scenario supplies an out-of-range feedback sample directly.
    MotorSnapshot snapshot;
    snapshot.state = MotorState::Running;
    snapshot.targetRpm = sim.motor.targetRpm();
    snapshot.driveCommandRpm = sim.motor.driveCommandRpm();
    snapshot.measuredRpm = MotorLimits::MaxRpm + 200;
    snapshot.controlErrorRpm = snapshot.targetRpm - snapshot.measuredRpm;
    snapshot.encoderUpdateMissing = false;
    sim.faults.evaluate(snapshot);

    check.expect(sim.faults.activeCode() == FaultCode::Overspeed, "overspeed was not detected");
    check.expect(sim.motor.state() == MotorState::Fault, "overspeed did not enter FAULT");
    check.expect(sim.motor.driveCommandRpm() == 0, "overspeed did not clear the drive command");
    reportScenario("Overspeed safe stop", check, failures);
}

void scenarioFaultLatch(int& failures) {
    Simulation sim;
    Check check;
    sim.motor.setTargetSpeed(800);
    sim.motor.start();
    runTicks(sim, MotorLimits::CommunicationTimeoutTicks, true);

    const CommandResult startAgain = sim.motor.start();
    applyResult(sim, startAgain);
    check.expect(startAgain == CommandResult::InvalidCommand, "start was accepted while faulted");
    check.expect(sim.faults.activeCode() == FaultCode::CommunicationTimeout,
                 "start replaced the latched fault");

    const CommandResult newSpeed = sim.motor.setTargetSpeed(100);
    applyResult(sim, newSpeed);
    check.expect(newSpeed == CommandResult::InvalidCommand, "speed was accepted while faulted");
    check.expect(sim.motor.targetRpm() == 800, "rejected speed changed the target");
    check.expect(sim.faults.activeCode() == FaultCode::CommunicationTimeout,
                 "speed replaced the latched fault");

    const CommandResult cleared = sim.motor.clearFault();
    check.expect(cleared == CommandResult::Accepted, "clear was rejected");
    check.expect(sim.motor.state() == MotorState::Stopped, "clear did not return to STOPPED");
    sim.faults.clear();
    check.expect(sim.faults.activeCode() == FaultCode::None, "fault status remained latched");

    const CommandResult restarted = sim.motor.start();
    check.expect(restarted == CommandResult::Accepted, "start after clear was rejected");
    check.expect(sim.motor.state() == MotorState::Running, "motor did not recover to RUNNING");
    reportScenario("Fault latch and recovery", check, failures);
}

int runAllScenarios() {
    std::cout << "Running scenarios (simulation only)\n";
    int failures = 0;
    scenarioStartStop(failures);
    scenarioValidSpeeds(failures);
    scenarioRejectInvalid(failures);
    scenarioEncoderAndError(failures);
    scenarioExcessiveError(failures);
    scenarioTimeout(failures);
    scenarioOverspeed(failures);
    scenarioFaultLatch(failures);
    std::cout << "Completed with " << failures << " failed scenario(s).\n";
    return failures;
}

bool hasExtraToken(std::istringstream& input) {
    std::string extra;
    return static_cast<bool>(input >> extra);
}

}  // namespace

int main() {
    std::cout << "Motor control simulation\n"
              << "Software model only. No physical motor is controlled.\n"
              << "Type 'help' for commands.\n";

    Simulation sim;
    std::string line;
    while (std::cout << "> " && std::getline(std::cin, line)) {
        std::istringstream input(line);
        std::string command;
        if (!(input >> command)) {
            continue;
        }

        if (command == "quit") {
            break;
        }
        if (command == "help") {
            printHelp();
            continue;
        }
        if (command == "status") {
            printStatus(sim);
            continue;
        }
        if (command == "test") {
            runAllScenarios();
            continue;
        }
        if (command == "start") {
            if (hasExtraToken(input)) {
                std::cout << "Usage: start\n";
                continue;
            }
            const CommandResult result = sim.motor.start();
            applyResult(sim, result);
            printCommandOutcome(sim, result);
            continue;
        }
        if (command == "stop") {
            if (hasExtraToken(input)) {
                std::cout << "Usage: stop\n";
                continue;
            }
            const CommandResult result = sim.motor.stop();
            applyResult(sim, result);
            printCommandOutcome(sim, result);
            continue;
        }
        if (command == "speed") {
            std::string rpmText;
            int rpm = 0;
            if (!(input >> rpmText) || hasExtraToken(input) || !parseInt(rpmText, rpm)) {
                const CommandResult result = CommandResult::InvalidCommand;
                applyResult(sim, result);
                printCommandOutcome(sim, result);
                continue;
            }
            const CommandResult result = sim.motor.setTargetSpeed(rpm);
            applyResult(sim, result);
            printCommandOutcome(sim, result);
            continue;
        }
        if (command == "tick") {
            std::string countText;
            int count = 1;
            if (input >> countText) {
                if (hasExtraToken(input) || !parseInt(countText, count) ||
                    count < 1 || count > MaxTicksPerCommand) {
                    std::cout << "Usage: tick [1-1000]\n";
                    continue;
                }
            }
            runTicks(sim, count, false);
            printStatus(sim);
            continue;
        }
        if (command == "timeout") {
            if (hasExtraToken(input)) {
                std::cout << "Usage: timeout\n";
                continue;
            }
            runTicks(sim, MotorLimits::CommunicationTimeoutTicks, true);
            printStatus(sim);
            continue;
        }
        if (command == "clear") {
            if (hasExtraToken(input)) {
                std::cout << "Usage: clear\n";
                continue;
            }
            const CommandResult result = sim.motor.clearFault();
            if (result == CommandResult::Accepted) {
                sim.faults.clear();
                std::cout << "Fault cleared. Motor is STOPPED.\n";
            } else {
                applyResult(sim, result);
                std::cout << sim.faults.message() << '\n';
            }
            printStatus(sim);
            continue;
        }

        std::cout << "Unknown command. Type 'help' for commands.\n";
    }

    return 0;
}
