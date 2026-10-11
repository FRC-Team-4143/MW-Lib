# MW-Lib - MARS/WARS Robotics Program FRC Library

A Java library for FRC (FIRST Robotics Competition) teams, providing utilities for swerve drive, mechanisms, geometry, and more.

📖 **API docs:** https://frc-team-4143.github.io/MW-Lib/

## Features

- **Swerve Drive Library**: Complete swerve drive implementation with Phoenix 6 support
- **Mechanism Classes**: Base classes for arms, elevators, flywheels, and rollers
- **Geometry Utilities**: Regions, splines, and geometric calculations
- **Logging Integration**: Built-in support for elastic logging
- **Proxy Server**: Communication utilities for robot data
- **Behavior-Tree Autos**: Autos written as BehaviorTree.CPP v4 XML, with shared nodes, tunable
  parameters, run logs and a live WebSocket view (`com.marswars.bt`)
- **Robot Identity**: Per-robot selection via a burned "RobotName" preference (constants live in
  each robot project's Java classes)

## Installation

### Option 1: Vendor Dependency (Recommended)

1. Download the `MWLib.json` vendordep file from the [latest release](https://github.com/FRC-Team-4143/MW-Lib/releases/latest)
2. Place it in your robot project's `vendordeps/` directory
3. Refresh your Gradle project

### Option 2: Manual Installation

Add to your `build.gradle` dependencies:

```gradle
dependencies {
    implementation 'com.github.frc-team-4143:mw-lib-java:27.0.0'
}
```

And add the Maven repository (GitHub Packages needs a token with `read:packages`):

```gradle
repositories {
    maven {
        name = "GitHubPackages"
        url = "https://maven.pkg.github.com/FRC-Team-4143/MW-Lib"
        credentials {
            username = project.findProperty("gpr.user") ?: System.getenv("USERNAME")
            password = project.findProperty("gpr.key") ?: System.getenv("TOKEN")
        }
    }
}
```

Projects that can't hold a token (the Scriptum lessons, for example) use the same release through
JitPack, which builds every `vX.Y.Z` tag with no credentials:

```gradle
repositories { maven { url = 'https://jitpack.io' } }
dependencies { implementation 'com.github.FRC-Team-4143.MW-Lib:mw-lib-java:27.0.0' }
```

Versions are `YY.minor.patch`: the first number is the season the architecture is written for (`27.0.0` opens the 2027 season), the second a feature release within it, the third a fix. Tags carry a `v`.
Robot projects that want to build against a local checkout instead of a release use mainbot's
`local.properties` / `mwlibLocal=true` composite build (see its README).

**Note**: For GitHub Packages, you need a GitHub Personal Access Token with `read:packages` permission.

## Building from Source

```bash
# Clone the repository
git clone <repository-url>
cd MW-Lib

# Build the library
./gradlew build

# Build vendor dependency JSON
./gradlew vendordepJson

# Build all artifacts
./gradlew outputJar outputSourcesJar

# Publish to local repository (for testing)
./gradlew publishToMavenLocal
```

## Development & Publishing

This library uses GitHub Actions for automated CI/CD:

### Continuous Integration
- **CI builds** run on every push and pull request
- **Tests** are executed against Java 17 and 21
- **Build artifacts** are generated and uploaded

### Publishing Releases
Releases are cut from `develop` by the **Make Release** workflow; nothing is tagged by hand.

1. Merge the work into `develop` (PR, CI green).
2. **Actions** → **Make Release** → **Run workflow**, enter the version (`27.0.0`, no `v`).
3. The workflow builds `develop` with that version, runs the tests, publishes `mw-lib-java` to
   GitHub Packages, tags `v27.0.0`, creates the GitHub Release (JARs attached, release notes from
   the commits) and publishes the Javadoc to `https://frc-team-4143.github.io/MW-Lib/`.
4. JitPack builds the new tag on first request; check
   `https://jitpack.io/com/github/FRC-Team-4143/MW-Lib/mw-lib-java/27.0.0/build.log`.

Put breaking changes (removed classes, renamed keys) in the PR description so they land in the
release notes. `publish.sh` / `PUBLISHING.md` cover publishing a local build by hand for testing.

## Usage

The library is organized into several packages:

- `com.marswars.swerve_lib` - Swerve drive implementation
- `com.marswars.mechanisms` - Mechanism base classes
- `com.marswars.geometry` - Geometric utilities
- `com.marswars.logging` - Logging utilities
- `com.marswars.util` - General utilities
- `com.marswars.subsystem` - Subsystem state machines and the robot super state machine (see [State machines](#state-machines))
- `com.marswars.bt` - Behavior-tree engine for autos (see [Behavior-tree autos](#behavior-tree-autos))

Example usage:

```java
import com.marswars.swerve_lib.MwSwerveSubsystem;
import com.marswars.swerve_lib.SwerveDriverInputs;
import com.marswars.mechanisms.ArmMech;
import com.marswars.util.RobotIdentity;

// Resolve which robot the code is running on (burned "RobotName" preference,
// or SimBot/ROBOT_NAME env var in simulation), mapped onto the project's own enum
// ("BetaBot" -> BETA_BOT); constants variants switch on it
public enum Robot { ALPHA_BOT, BETA_BOT, SIM_BOT }
Robot robot = RobotIdentity.getInstance().getRobot(Robot.class);

// Swerve drive: extend MwSwerveSubsystem with your constants (extending MwSwerveConstants),
// a field-pose supplier and the driver joystick inputs. The constants' getDriveConfig() is
// built with SwerveDriveConfig.builder() (module type, wheel radius, gains, CAN IDs, positions).
public class SwerveSubsystem extends MwSwerveSubsystem<SwerveConstants> {
    public SwerveSubsystem() {
        super(SwerveConstants.create(), localization::getFieldPose,
                new SwerveDriverInputs(oi::leftX, oi::leftY, oi::rightX, oi::pov));
    }
}

// Use arm mechanism
ArmMech arm = new ArmMech(config);
```

## State machines

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
        switch (getSystemState()) {
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

## Behavior-tree autos

`com.marswars.bt` runs autos written as [BehaviorTree.CPP v4](https://www.behaviortree.dev/) XML
files, which open in [Arbor](https://github.com/coalman321/Arbor) (our tree editor and live viewer) and Groot2. Each `<Name>.xml` file in a robot project's autos
folder becomes an auto in the chooser, except library files (several trees and no
`main_tree_to_execute`, such as a shared `common.xml`), which autos pull in with `<include>`. mainbot's `src/main/deploy/autos/README.md` is
the guide to writing them.

**Setting it up in a robot project:**

```java
BehaviorTreeFactory factory = new BehaviorTreeFactory();      // BT.CPP built-ins + MW-Lib nodes
SwerveNodes.register(factory, SwerveSubsystem::getInstance);  // MW-Lib swerve nodes
RobotStateNodes.register(factory, RobotContainer::getInstance, RobotStates.class); // SetWantedState

// deploy/autos is an Arbor project: bt_project.json, nodes.xml and the trees/ folder
AutoManager.getInstance().registerAutos(BehaviorTreeAuto.loadAll(
        factory, Filesystem.getDeployDirectory().toPath().resolve("autos/trees")));
BtLiveServer.start(BtLiveServer.Options.defaults());     // live view for Arbor (port 1670)
BehaviorTreeFileLogger.enableDefault();                  // one .btlog (FileLogger2 format) per run
```

**What's included:**

| Piece | What it is |
|---|---|
| `bt.core`, `bt.control`, `bt.decorator`, `bt.action` | The engine, with BT.CPP v4 node semantics and all BT.CPP built-ins except scripting |
| `bt.xml` | BT.CPP v4 XML parser and writer: both node forms, `SubTree` port passing, `<include>`, `<TreeNodesModel>` |
| `ParallelDeadline` | MW-Lib control node: runs children together until the first one finishes, then halts the rest |
| `bt.RobotStateNodes` | `SetWantedState` and `IsSystemState` for the robot's super state machine. Autos use the same robot states and sequences as teleop |
| `bt.swerve.SwerveNodes` | `FollowTrajectory`, `WaitForChoreoEvent`, `SetSwerveState` and Choreo/chassis conditions for any `MwSwerveSubsystem` |
| `auto.BehaviorTreeAuto` | An `Auto` loaded from XML. It pre-loads and alliance-flips every trajectory the XML names, and re-reads the file when the auto is re-selected |
| Tree parameters | Ports declared for a tree in `<TreeNodesModel>` become dashboard values at `/Tuning/Autos/<auto>/<port>`, read into the blackboard each run |
| `bt.monitor` | Logs `BehaviorTree/<auto>/{Status,Structure,Xml,Result}` through `MwLog`. `BehaviorTreeFileLogger` writes a `.btlog` per run in BT.CPP's `FileLogger2` format, with every status transition, for replay in Arbor or Groot2 |
| `bt.debug.BtLiveServer` | Streams the running tree over WebSocket using the btlive v1 protocol (default port 1670) |

**Node palette.** MW-Lib's shared nodes ship as `com/marswars/bt/mwlib_nodes.xml`. A robot's
palette is that file with the robot's own nodes appended, from
`factory.writeTreeNodesModelXml(false)`. Regenerate MW-Lib's copy after changing a shared node:

```
./gradlew test -PupdateNodeSpec
```

Generic nodes any robot could use belong in MW-Lib, registered with `registerLibraryNode`. Robot
projects register only their own.

## Dependencies

This library depends on:
- WPILib 2026+
- Phoenix 6 (CTRE)
- Jackson (JSON processing)
- Various vendor libraries (Maplesim, Playing With Fusion, etc.)
- Java-WebSocket 1.6.0, for the behavior-tree live server. MW-Lib's published package doesn't list
  its dependencies, so robot projects must add
  `implementation 'org.java-websocket:Java-WebSocket:1.6.0'` themselves.

## License

This project is licensed under the MIT License - see the [LICENSE.txt](LICENSE.txt) file for details.

## Library Components

### Subsystems
- `MwSubsystem` - Base subsystem class (a state machine) with logging integration
- `MwSuperStateMachine` - The robot's super state machine; `RobotContainer` extends it (see [State machines](#state-machines))
- `SubsystemManager` - Centralized subsystem management

### Mechanisms
- `ArmMech` - Multi-motor arm mechanism with position/velocity control
- `ElevatorMech` - Linear elevator mechanism
- `FlywheelMech` - Velocity-controlled flywheel
- `RollerMech` - Simple roller mechanism

