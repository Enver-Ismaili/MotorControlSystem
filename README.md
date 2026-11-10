# Motor Control Simulation

This is a C++17 console simulation of a single-motor control loop. It models command handling, encoder feedback, a proportional speed step, and fault monitoring.

It does not control a physical motor. It does not generate PWM, read a real encoder, or communicate with a drive. Nothing in this project provides real-time behavior.

## Purpose

The project is a small portfolio example of how a junior embedded engineer might separate command handling, feedback, control, and fault response into testable modules. The hardware behind those modules is simulated in software so the program can be built and run in Visual Studio.

## Architecture

```
include/                 src/
  MotorTypes.h             main.cpp
  MotorController.h        MotorController.cpp
  EncoderSimulator.h       EncoderSimulator.cpp
  FaultMonitor.h           FaultMonitor.cpp
```

`main.cpp` owns one `MotorController`, one `EncoderSimulator`, and one `FaultMonitor`. A command or a software tick moves data in one direction:

1. The controller accepts or rejects start, stop, and target speed.
2. On each tick the encoder simulation moves its feedback toward the drive command.
3. The controller applies one proportional step from the speed error.
4. The fault monitor checks that result and, for a critical fault, calls a safe stop.

Shared limits and enums live in `include/MotorTypes.h`.

| Limit | Value |
| --- | --- |
| Speed command | 0 to 3000 RPM |
| Proportional gain | 4/5 of the speed error, added to the drive command each tick |
| Encoder lag | closes 7/10 of the gap to the drive command each tick |
| Encoder ripple | repeating offsets of -18, -6, 0, and -12 RPM |
| Excessive error | more than 500 RPM for 5 ticks while RUNNING |
| Communication timeout | 3 consecutive ticks with no encoder update |

## Features

- States are `STOPPED`, `RUNNING`, and `FAULT`.
- `start` is accepted only from `STOPPED`. `stop` is a normal stop and is accepted only from `RUNNING`.
- Target speed is stored only when it is inside 0 to 3000 RPM. A rejected speed leaves the previous target unchanged.
- While the motor is `RUNNING`, each tick updates the drive command with `drive += (4/5) * (target - measured)`, then clamps it to 0..3000 RPM.
- The encoder is a first-order lag plus a fixed ripple. The same command sequence always produces the same feedback.
- Invalid commands and out-of-range speeds are recorded and do not change the motor state.
- Critical faults are communication timeout, overspeed, and excessive speed error. Each one forces a safe stop: state `FAULT`, drive command 0. The fault stays latched until `clear`.
- `clear` returns the motor to `STOPPED` and keeps the last target. `start` is then accepted again.

Actual RPM on the status display is the simulated encoder value. There is no second measurement source.

## Technologies

- C++17
- Microsoft Visual C++ (MSVC), platform toolset v145
- Visual Studio console project (`MotorControlSystem.slnx`)
- Standard C++ library only

## Build and run

Open `MotorControlSystem.slnx` in Visual Studio, choose **Debug | x64**, and start without debugging.

From a Developer Command Prompt for Visual Studio, in this directory:

```
msbuild MotorControlSystem.vcxproj /p:Configuration=Debug /p:Platform=x64
x64\Debug\MotorControlSystem.exe
```

The project language standard is C++17. Headers are included from `include/`.

## Commands

Each `tick` is one software step. It is not a hardware control period.

| Command | Action |
| --- | --- |
| `start` | Enter `RUNNING` from `STOPPED` |
| `stop` | Normal stop from `RUNNING` |
| `speed <rpm>` | Set the target, from 0 to 3000 |
| `status` | Print state, speeds, error, and fault |
| `tick [n]` | Run 1 to 1000 control steps |
| `timeout` | Skip 3 encoder updates |
| `clear` | Clear a latched fault and return to `STOPPED` |
| `test` | Run the scenarios below without changing the live session |
| `help` | List the commands |
| `quit` | Exit |

## Example console output

This session was produced by the Debug x64 build. The drive command stays 0 until the first tick after `start`. After a timeout, the last encoder sample remains on screen because those ticks do not take a new sample, and the drive command is forced to 0.

```
Motor control simulation
Software model only. No physical motor is controlled.
Type 'help' for commands.
> speed 1200
Command accepted.
Status: STOPPED
Target RPM: 1200
Actual RPM: 0
Encoder feedback RPM: 0
Drive command RPM: 0
Control error RPM: 0
Fault: NONE
Message: No fault.
> start
Command accepted.
Status: RUNNING
Target RPM: 1200
Actual RPM: 0
Encoder feedback RPM: 0
Drive command RPM: 0
Control error RPM: 0
Fault: NONE
Message: No fault.
> tick 8
Status: RUNNING
Target RPM: 1200
Actual RPM: 1178
Encoder feedback RPM: 1178
Drive command RPM: 1198
Control error RPM: 22
Fault: NONE
Message: No fault.
> timeout
Status: FAULT
Target RPM: 1200
Actual RPM: 1178
Encoder feedback RPM: 1178
Drive command RPM: 0
Control error RPM: 22
Fault: COMMUNICATION_TIMEOUT
Message: Encoder feedback was not received.
> clear
Fault cleared. Motor is STOPPED.
Status: STOPPED
Target RPM: 1200
Actual RPM: 1178
Encoder feedback RPM: 1178
Drive command RPM: 0
Control error RPM: 22
Fault: NONE
Message: No fault.
```

`speed -1` or `speed 3001` instead prints `Target speed must be from 0 to 3000 RPM.` and sets fault `SPEED_OUT_OF_RANGE`. The stored target does not change.

## Testing

Type `test` at the prompt. The scenarios use the same classes as the console and print `[PASS]` or `[FAIL]`.

- Start, then stop, ends in `STOPPED` with drive command 0.
- Targets 500, 1500, 3000, and 0 are stored. After start, ticks use 1500 RPM and produce a non-zero drive command and encoder reading.
- Negative RPM, RPM above 3000, stop while already stopped, and a second start are rejected.
- Ten ticks at 1200 RPM move the encoder toward the target and shrink the control error.
- A stalled encoder holds feedback near 0. After 5 ticks the monitor raises `EXCESSIVE_ERROR`, the state is `FAULT`, and the drive command is 0.
- Two missed encoder updates do not fault. The third raises `COMMUNICATION_TIMEOUT` and safe-stops.
- Feedback reported as 3200 RPM raises `OVERSPEED` and safe-stops. Normal encoder ripple is clamped to 0..3000 RPM, so this case supplies the illegal sample directly.
- While a critical fault is latched, start and speed are rejected and the original fault code remains. `clear` then `start` returns the motor to `RUNNING`.

A Debug x64 run of `test` completed with 0 failed scenarios.

## Limitations

- This is a software simulation of one motor. It is not motor firmware.
- There is no GPIO, PWM, ADC, encoder timer, CAN, serial port, or other hardware interface.
- Ticks run only when the console asks for them. There is no scheduler and no real-time guarantee.
- The control law is proportional only. The drive command is an integer software value, not a current or voltage command.
- Encoder "measurements" are a lag filter plus a repeating offset. They are not a mechanical model and they do not include load, inertia, or noise from a real sensor.
- The excessive-error scenario stalls that filter on purpose so the error cannot close. The stall hook is not a user command.
- Overspeed is detected when a feedback sample is above 3000 RPM. The normal encoder path does not produce that sample.
- Safe stop means the simulated state becomes `FAULT` and the simulated drive command becomes 0. It does not remove power from equipment.
