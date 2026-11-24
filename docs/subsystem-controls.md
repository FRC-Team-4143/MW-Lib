# Subsystem modes

Every subsystem registered with `SubsystemManager` (or `MwSuperStateMachine`) can be switched live
between three modes, and each of its mechanisms switched off on its own, with no robot code.

| Mode       | What happens |
|------------|--------------|
| `ENABLED`  | Normal. The subsystem's logic runs every loop. |
| `DISABLED` | The logic doesn't run. Every mechanism holds its motors neutral (each motor's configured brake or coast). Inputs are still read and logged. |
| `TUNING`   | The logic doesn't run, so nothing but you moves the subsystem. On entry each mechanism holds where it is (velocity mechanisms stop). After that, the mechanism's existing setpoint tunables drive it. |

## NetworkTables

| Entry | Written by | Holds |
|---|---|---|
| `/MwLib/Subsystems/<Name>/Mode` | dashboard | `ENABLED`, `DISABLED` or `TUNING` |
| `/MwLib/Subsystems/<Name>/ActiveMode` | robot | the mode it applied |
| `/MwLib/Subsystems/<Name>/Mechs/<Mech>/Enabled` | dashboard | `false` holds that one mechanism neutral |
| `/MwLib/Subsystems/<Name>/Mechs/<Mech>/LoggingKey` | robot | where the mech's tunables and inputs live |

`<Name>` is the subsystem's class name without `Subsystem`. A mechanism switched off stays off
through mode changes until it is switched back on. Every boot starts with everything enabled.

## How it works (and what it costs)

The manager keeps two lists: **running** (ENABLED; logic runs) and **parked** (DISABLED or
TUNING; logic skipped). Each loop goes through both lists; parked subsystems still read inputs,
write outputs and log.

A switch costs nothing per loop. Writing an entry fires an NT listener, which queues the change.
The manager applies queued changes once at the top of its next loop, on the robot thread:

- **To DISABLED:** every IO's `setNeutralOutputs(true)`; the subsystem moves to the parked list.
- **To TUNING:** every mechanism's `hold()`; the subsystem moves to the parked list.
- **To ENABLED:** outputs un-neutraled, then `onEnable()` (`reset()`, and the wanted state back to
  the default); the subsystem moves back to the running list.
- **A mechanism on or off:** that mechanism's `setNeutralOutputs`. One switched back on in TUNING
  is held, so it doesn't jump to a stale target.

The only per-loop cost is checking that the queue is empty, plus one boolean read in each
mechanism's `writeOutputs`.

## Tuning

TUNING uses what the mechanisms already have. Each `*Mech` registers live setpoint tunables:
`PositionGains/Setpoint`, `VelocityGains/Setpoint`, `DutyCycle/Setpoint` and
`CurrentGains/Setpoint`, under `/Tuning/<LoggingKey>`. Each calls the matching `setTarget*` when its
value changes. With the subsystem parked, nothing overwrites those targets, so they drive the
mechanism. Gains are tuned the same way as always, in `/Tuning`.

A tunable only fires when its value changes, so to send the same setpoint again, change it and
change it back. `SwerveMech` has no setpoint tunables, so it holds still in TUNING; tune the drive
with `MwSwerveSubsystem.chassisTuningCommand` as before.

## Matches (FMS attached)

TUNING is refused, and the dashboard entry is put back. `reset()` (call it from your mode inits, as
usual) moves anything still in TUNING back to ENABLED. DISABLED still works, so a broken mechanism
or subsystem can be switched off mid-match.

## From code

All optional:

```java
startSubsystemConsole(SubsystemConsole.DEFAULT_PORT); // web control panel, below

operator.start().onTrue(Commands.runOnce(() -> requestMode(elevator, SubsystemMode.DISABLED)));
requestMechEnabled(intake, intake_rollers, false);

if (!isAvailable(elevator)) { /* pick another plan */ }
inMode(shooter, SubsystemMode.DISABLED).onTrue(leds.flash(Color.kRed));
```

Requests from code go through the same NT entries, so dashboards show them, and they apply at the
top of the next loop.

A custom IO (not a `*Mech`) with actuators should override `SubsystemIoBase.setNeutralOutputs` to
neutral them; otherwise it keeps applying its last command while its subsystem is DISABLED.
Subsystems can override `onEnable()` for anything beyond `reset()`.

## Subsystem console

`startSubsystemConsole(5807)` serves a control page bundled in MW-Lib from the robot. Browse to
`http://10.TE.AM.2:5807/` (or `http://localhost:5807/` in simulation). It talks NT4 to the robot
directly, so it needs no internet and nothing to deploy. It shows every subsystem's mode and state,
each mechanism's switch and measured position and velocity, and in TUNING the mechanism's setpoint
tunables. It doesn't show or edit gains. **Demo robot** runs a simulated robot in the page.

## Autos

`RobotStateNodes` adds the condition `IsSubsystemAvailable subsystem="Elevator"` (true when
ENABLED), so a tree can fail fast instead of waiting on a parked subsystem:

```xml
<ReactiveSequence>
  <IsSubsystemAvailable subsystem="Elevator"/>
  <Sequence> ... </Sequence>
</ReactiveSequence>
```

## Logs and replay

Mode and mechanism changes are logged as events (`Subsystem/<Name>/Mode`,
`Subsystem/<Name>/Mechs/<Mech>/Enabled`) and to the console. They are not replay inputs: a replay
runs every subsystem's logic.

## Build-time disable

The `disabled_subsystems` constructor list still leaves subsystems out entirely.
`getDisabledSubsystems()` returns it; `getEnabledSubsystems()` is deprecated because it returned
that same disabled list.
