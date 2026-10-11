# State machines

Every MW-Lib subsystem is a state machine (`MwSubsystem`). The robot as a whole is one more state
machine on top of them: the **super state machine**. A robot's `RobotContainer` extends
`MwSuperStateMachine<RobotStates>`, which coordinates the subsystems. Each loop:

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
public class RobotContainer extends MwSuperStateMachine<RobotStates> {
    public RobotContainer() {
        super(RobotStates.DRIVE, BuildConstants.class);
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
                    case SCORE -> {
                        if (shooter.hasPiece()) system_state_ = RobotStates.AIM;
                        else failWantedState(); // autos see FAILURE
                    }
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
<!-- auto: the same sequence. The node waits for SUCCESS or FAILURE, and the tree adds auto-only fail handling -->
<Fallback>
  <Timeout msec="3000"><SetWantedState state="SCORE"/></Timeout>
  <FollowTrajectory trajectory="ToBackupSpot"/>
</Fallback>
```

| API | What it does |
|---|---|
| `setWantedState(s)` / `getWantedState()` / `getSystemState()` | Same as `MwSubsystem` |
| `handleStateTransition(wanted)` | The transition table: every condition lives here (guards, step states, decided values). No enter/exit actions. The default goes straight to `wanted` |
| `getStateResult()` | `SUCCESS` once the robot is in the wanted state, `FAILURE` after `failWantedState()`, otherwise `RUNNING` |
| `failWantedState()` | Report that the wanted state can't be reached. Cleared by the next `setWantedState` |
| `isIn(s)` / `inState(s)` | Is the robot in `s`. `inState` returns a `Trigger` |
| `bt.RobotStateNodes` | Behavior-tree nodes `SetWantedState` (requests the state and waits on `getStateResult()`) and `IsSystemState` |

MW-Lib subsystems are not WPILib `Subsystem`s, so button commands have no requirements. Logged
keys are `Robot/WantedState` and `Robot/State`.
