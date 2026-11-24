# Subsystem controls

Every subsystem registered with `SubsystemManager` (or `MwSuperStateMachine`) can be switched,
pinned, overridden or turned off live from NetworkTables, with no robot code. Every library IO in
it (each `*Mech`, `Tof`, `Gyro`, and the gyro inside `SwerveMech`) gets its own on/off switch,
sensor overrides and an output override.

## Who controls a subsystem

`/MwLib/Subsystems/<Name>/Control`, where `<Name>` is the class name without `Subsystem`:

| Control  | What runs                                                                                     |
|----------|-----------------------------------------------------------------------------------------------|
| `ROBOT`  | The default. The subsystem follows `setWantedState` from the super state machine, autos and buttons. |
| `PINNED` | Held in the state at `/MwLib/Subsystems/<Name>/PinnedState`. Its own transition and logic run; requests are dropped. |
| `MANUAL` | Its logic does not run. Each mechanism follows its output override (below).                    |
| `OFF`    | Its logic does not run and every motor goes to its configured neutral (brake or coast). Inputs are still read and logged. |

- Requests to a subsystem that is not `ROBOT` are dropped and logged as `DroppedWantedState`.
- Going back to `ROBOT`, or leaving `OFF` or `MANUAL` for a control that runs the logic, calls
  `reset()` and returns the wanted state to the default.
- `/MwLib/Subsystems/<Name>/ActiveControl` is what the robot applied; `/States` lists the states a
  dashboard can offer for pinning.
- Every boot starts at `ROBOT`. A subsystem not under robot control raises a warning alert; an
  unknown control or state name raises an error alert and falls back to `ROBOT`.

## Per-IO controls

`/MwLib/Subsystems/<Name>/IO/<IoName>/`:

| Entry | Effect |
|---|---|
| `Enabled` | `false` switches the IO off. A mechanism holds its motors neutral while the rest of the subsystem keeps running. A gyro reports disconnected (swerve falls back to kinematics); a ToF reports nothing in range. |
| `Override/<Channel>/Active`, `Value` | Replaces a reading in the IO's getters. Channels: `Position`, `Velocity` (mechanisms), `Range` (ToF, meters), `Yaw`, `Pitch`, `Roll` (gyro, degrees). The raw reading is still logged. |
| `Manual/Mode`, `Manual/Setpoint` | The output while the subsystem is `MANUAL`. `Manual/Modes` lists what the mechanism supports: `NEUTRAL`, `POSITION`, `MOTION_PROFILE_POSITION`, `VELOCITY`, `MOTION_PROFILE_VELOCITY`, `DUTY_CYCLE`, `CURRENT`. Setpoints use the units of the matching `setTarget*` method. |
| `Manual/Step`, `StepTo`, `StepPeriod` | Alternates the setpoint between `Setpoint` and `StepTo` every `StepPeriod` seconds. |

- On entering `MANUAL`, position mechanisms hold where they are and velocity mechanisms start at zero.
- Duty cycle is clamped to ±0.30 (`setManualDutyCycleLimit`).
- Swerve in `MANUAL`: `POSITION` points every wheel at an angle (radians), `VELOCITY` drives
  straight forward, robot-relative (m/s).
- Names clash inside a subsystem get a number (`Roller`, `Roller2`).

## Tunables

A library IO's tunables (keys under its logging key, such as motor gains) only accept changes while
its subsystem is in `MANUAL`; changes at other times are put back. Other tunables, including
subsystem-level gains that need the logic running, stay live. Leaving `MANUAL` logs every changed
value as `Subsystem/<Name>/TunedValues` and to the console, to copy into constants.
`setTuneOnlyInManual(false)` turns the gate off.

## Matches (FMS attached)

`PINNED`, `MANUAL`, sensor overrides and every tunable are ignored. `OFF`, for a subsystem or a
single IO, still works so a broken mechanism can be shut off; `setAllowOffWhenFmsAttached(false)`
locks it too.

## Replay

Every dashboard entry is read as an AdvantageKit input, so log replay reproduces every control
change, override and output override.

## From code

All optional. In `RobotContainer`:

```java
setAllowOffWhenFmsAttached(true);      // default
setManualDutyCycleLimit(0.30);         // default
setTuneOnlyInManual(true);             // default

startSubsystemConsole(SubsystemConsole.DEFAULT_PORT); // web console, below
new TuningGamepad(this, 3);                           // spare controller, below

// switches, same as a dashboard write (applies next loop)
operator.start().onTrue(requestControlCommand(elevator, SubsystemControl.OFF));
requestPinnedState(elevator, ElevatorStates.HIGH);

// checks
if (!isAvailable(elevator)) { /* pick another plan */ }
inControl(shooter, SubsystemControl.OFF).onTrue(leds.flash(Color.kRed));
getIoControls(elevator_mech).requestManual(ManualMode.POSITION, 0.5);
```

Mechanisms also expose `isAvailable()` (false while switched off).

## Subsystem console

`startSubsystemConsole(5807)` serves a web page bundled in MW-Lib from the robot: open
`http://10.TE.AM.2:5807/` (or `http://localhost:5807/` in simulation). It talks NT4 to the robot
directly and needs no internet or deploy step. It shows every subsystem with its control and
state, and for the selected one each IO's on/off switch, sensor overrides and output override, with
what the robot is applying and measuring. It doesn't edit gains; use your usual dashboard for
`/Tuning`. The **Demo robot** button runs a simulated robot in the page.

## Tuning gamepad

`new TuningGamepad(this, port)` drives mechanisms whose subsystem is in `MANUAL` from a spare Xbox
controller, writing the same entries the console does:

- Bumpers: previous / next mechanism (shown in the console as "Gamepad: …")
- Left stick Y: duty cycle, scaled to the duty-cycle limit; on release the mechanism holds
- A: start or stop the step test
- B: neutral

## Autos

`RobotStateNodes` adds the condition `IsSubsystemAvailable subsystem="Elevator"`, so a tree can fail
fast rather than wait on a subsystem that was pinned, put in manual or switched off:

```xml
<ReactiveSequence>
  <IsSubsystemAvailable subsystem="Elevator"/>
  <Sequence> ... </Sequence>
</ReactiveSequence>
```

## Build-time disable

The `disabled_subsystems` constructor list still leaves subsystems out entirely (not registered).
`getDisabledSubsystems()` returns it; `getEnabledSubsystems()` is deprecated because it returned
that same disabled list.
