#pragma once

// Stand-in for a rotary encoder. It does not sample a timer, count pulses,
// or read a sensor. Each update moves a simulated measurement toward the
// drive command and adds a fixed repeating offset so the two values differ.
class EncoderSimulator {
public:
    // Holds the lag state so feedback stops following the drive command.
    // Used to demonstrate a sustained speed error. Communication still occurs.
    void setStalled(bool stalled);
    bool stalled() const;

    int update(int driveCommandRpm);
    int measuredRpm() const;

private:
    int rippleOffset();

    int lagRpm_ = 0;
    int reportedRpm_ = 0;
    int tick_ = 0;
    bool stalled_ = false;
};
