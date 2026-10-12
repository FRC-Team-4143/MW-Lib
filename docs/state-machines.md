# State machines

Every MW-Lib subsystem is a state machine (`MwSubsystem`). The robot as a whole is one more state
machine on top of them: the **super state machine**. A robot's `RobotContainer` extends
`MwSuperStateMachine<RobotStates, RobotConstants>`, which coordinates the subsystems. It is written
like a big subsystem: a constants class (`CONSTANTS`) holding the states enum, `system_state_`,
`handleStateTransition` and `updateLogic`. Each loop:

1. **`RobotContainer` (super state machine).** It moves to the wanted robot state through
   `handleStateTransition`. Then `updateLogic` requests a state from each subsystem with
   `setWantedState(...)`.
2. **Every registered subsystem.** It reads inputs, moves to its own wanted state, runs its logic
   and writes outputs.

The subsystems see the super state's requests in the same loop.

Buttons and autos call `setWantedState(...)` on the robot, the same way the robot calls it on
subsystems. There are no per-subsystem command files. Logic that involves more than one subsystem
lives in one place, the super state machine, and teleop and autos share it. A scoring sequence is
written once, as a robot state, and both teleop and autos use it.

```java
// The robot's states live in RobotConstants, the way a subsystem's states live in its
// *Constants class. A new robot can start with just DRIVE.
public class RobotConstants extends MwConstants {
    public enum RobotStates { IDLE, DRIVE, AIM, SPIN_UP, SCORE }
}

public class RobotContainer extends MwSuperStateMachine<RobotStates, RobotConstants> {
    public RobotContainer() {
        super(RobotStates.DRIVE, new RobotConstants(), BuildConstants.class);
        registerSubsystem(DrivetrainSubsystem.getInstance());
        registerSubsystem(ShooterSubsystem.getInstance());
        reset(); // leave as the last line
    }

    // The transition table: switch on where we are, then on what was asked. Every condition
    // lives here; updateLogic below has none. Wanting SCORE walks through the step states
    // AIM and SPIN_UP first (nothing requests those directly).
    @Override
    protected void handleStateTransition(RobotStates wanted) {
        switch (system_state_) {
            case AIM -> {
                switch (wanted) {
                    case SCORE -> { if (drive.isAimed()) system_state_ = RobotStates.SPIN_UP; }
                    case DRIVE, IDLE -> system_state_ = wanted;
                    default -> {}
                }
            }
            case SPIN_UP -> {
                switch (wanted) {
                    case SCORE -> { if (shooter.isReady()) system_state_ = RobotStates.SCORE; }
                    case DRIVE, IDLE -> system_state_ = wanted;
                    default -> {}
                }
            }
            case IDLE, DRIVE, SCORE -> {
                switch (wanted) {
                    case SCORE -> { if (shooter.hasPiece()) system_state_ = RobotStates.AIM; }
                    case DRIVE, IDLE -> system_state_ = wanted;
                    default -> {}
                }
            }
        }
    }

    @Override
    protected void updateLogic(double timestamp) {
        switch (system_state_) {
            case DRIVE -> { drive.setWantedState(DriveStates.ARCADE); shooter.setWantedState(ShooterStates.IDLE); }
            case AIM -> { drive.setWantedState(DriveStates.AIM); shooter.setWantedState(ShooterStates.IDLE); }
            case SPIN_UP -> { drive.setWantedState(DriveStates.AIM); shooter.setWantedState(ShooterStates.SPIN_UP); }
            case SCORE -> { drive.setWantedState(DriveStates.AIM); shooter.setWantedState(ShooterStates.SHOOT); }
        }
    }
}

// OI.configureBindings()
driver.rightBumper().whileTrue(Commands.startEnd(
        () -> robot.setWantedState(RobotStates.SCORE),
        () -> robot.setWantedState(RobotStates.DRIVE)));
```

```xml
<!-- auto: the same sequence. SetWantedState is an instant request, like the button press.
     To wait until the robot gets there, retry IsSystemState; the Timeout and Fallback add
     auto-only fail handling -->
<Sequence>
  <SetWantedState state="SCORE"/>
  <Fallback>
    <Timeout msec="3000">
      <RetryUntilSuccessful num_attempts="-1"><IsSystemState state="SCORE"/></RetryUntilSuccessful>
    </Timeout>
    <Sequence>
      <SetWantedState state="DRIVE"/>
      <FollowTrajectory trajectory="ToBackupSpot"/>
    </Sequence>
  </Fallback>
</Sequence>
```

| API | What it does |
|---|---|
| `setWantedState(s)` / `getWantedState()` / `getSystemState()` / `system_state_` | Same as `MwSubsystem` |
| `CONSTANTS` | The robot's constants class (`extends MwConstants`), like a subsystem's. It holds the `RobotStates` enum. Keep the robot identity in it as `static` members (`RobotConstants.ROBOT`): subsystems read it while `RobotContainer` is still being constructed |
| `handleStateTransition(wanted)` | The transition table: every condition lives here (guards, step states, decided values). No enter/exit actions. The default goes straight to `wanted` |
| `isIn(s)` / `inState(s)` | Is the robot in `s`. `inState` returns a `Trigger` |
| `bt.RobotStateNodes` | Behavior-tree nodes `SetWantedState` (an instant request: calls `setWantedState` and succeeds) and `IsSystemState` (the robot is in a state; retry it to wait) |

MW-Lib subsystems are not WPILib `Subsystem`s, so button commands have no requirements. Logged
keys are `Robot/WantedState` and `Robot/State`.
