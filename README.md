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
    implementation 'com.marswars.frc:MWLib-java:1.0.0'
}
```

And add the Maven repository:

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
1. **Create a tag**: `git tag v1.0.0 && git push origin v1.0.0`
2. **Create GitHub release** from the tag
3. **GitHub Action automatically**:
   - Builds the library with the release version
   - Publishes to GitHub Packages
   - Attaches `MWLib.json` vendor dependency to the release
    - Publishes Javadocs to GitHub Pages (`https://frc-team-4143.github.io/MW-Lib/`)

### Manual Publishing
You can also trigger publishing manually:
- Go to **Actions** → **Build and Publish** → **Run workflow**
- Check "Force publish to GitHub Packages"

## Usage

The library is organized into several packages:

- `com.marswars.swerve_lib` - Swerve drive implementation
- `com.marswars.mechanisms` - Mechanism base classes
- `com.marswars.geometry` - Geometric utilities
- `com.marswars.logging` - Logging utilities
- `com.marswars.util` - General utilities
- `com.marswars.bt` - Behavior-tree engine for autos (see [Behavior-tree autos](#behavior-tree-autos))

Example usage:

```java
import com.marswars.swerve_lib.MwSwerveSubsystem;
import com.marswars.swerve_lib.SwerveDriverInputs;
import com.marswars.mechanisms.ArmMech;
import com.marswars.util.RobotIdentity;

// Resolve which robot the code is running on (burned "RobotName" preference,
// or SimBot/ROBOT_NAME env var in simulation); robot projects map this name
// to their own Java constants variants
String robotName = RobotIdentity.getInstance().getRobotName();

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

## Behavior-tree autos

`com.marswars.bt` runs autos written as [BehaviorTree.CPP v4](https://www.behaviortree.dev/) XML
files, which open in Groot2 and the BT editor. Each `deploy/autos/<Name>.xml` file in a robot
project becomes an auto in the chooser. mainbot's `src/main/deploy/autos/README.md` is the guide
to writing them.

**Setting it up in a robot project:**

```java
BehaviorTreeFactory factory = new BehaviorTreeFactory();      // BT.CPP built-ins + MW-Lib nodes
SwerveNodes.register(factory, SwerveSubsystem::getInstance);  // MW-Lib swerve nodes
factory.registerSetState("SetIntakeState", "Request an intake state",
        IntakeStates.class, s -> IntakeSubsystem.getInstance().setWantedState(s));  // robot nodes

AutoManager.getInstance().registerAutos(BehaviorTreeAuto.loadAll(
        factory, Filesystem.getDeployDirectory().toPath().resolve("autos")));
BtLiveServer.start(BtLiveServer.Options.defaults());     // live view for the BT editor
BehaviorTreeFileLogger.enableDefault();                  // one .btlog.xml per auto run
```

**What's included:**

| Piece | What it is |
|---|---|
| `bt.core`, `bt.control`, `bt.decorator`, `bt.action` | The engine, with BT.CPP v4 node semantics and all BT.CPP built-ins except scripting |
| `bt.xml` | BT.CPP v4 XML parser and writer: both node forms, `SubTree` port passing, `<include>`, `<TreeNodesModel>` |
| `ParallelDeadline` | MW-Lib control node: runs children together until the first one finishes, then halts the rest |
| `bt.swerve.SwerveNodes` | `FollowTrajectory`, `WaitForChoreoEvent`, `SetSwerveState` and Choreo/chassis conditions for any `MwSwerveSubsystem` |
| `auto.BehaviorTreeAuto` | An `Auto` loaded from XML. It pre-loads and alliance-flips every trajectory the XML names, and re-reads the file when the auto is re-selected |
| Tree parameters | Ports declared for a tree in `<TreeNodesModel>` become dashboard values at `/Tuning/Autos/<auto>/<port>`, read into the blackboard each run |
| `bt.monitor` | Logs `BehaviorTree/<auto>/{Status,Structure,Xml,Result}` through `MwLog`. `BehaviorTreeFileLogger` writes a `.btlog.xml` per run with every status transition |
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
- `MwSubsystem` - Base subsystem class with logging integration
- `SubsystemManager` - Centralized subsystem management

### Mechanisms
- `ArmMech` - Multi-motor arm mechanism with position/velocity control
- `ElevatorMech` - Linear elevator mechanism
- `FlywheelMech` - Velocity-controlled flywheel
- `RollerMech` - Simple roller mechanism

