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

### Option 1: Vendor Dependency

1. Download the `MWLib.json` vendordep file from the [latest release](https://github.com/FRC-Team-4143/MW-Lib/releases/latest)
2. Place it in your robot project's `vendordeps/` directory
3. Refresh your Gradle project

### Option 2: JitPack (Recommended)

JitPack builds every `vX.Y.Z` tag on first request, with no token or credentials. Add to your `build.gradle`:

```gradle
repositories {
    maven { url = 'https://jitpack.io' }
}

dependencies {
    implementation 'com.github.FRC-Team-4143.MW-Lib:mw-lib-java:27.0.0'
}
```

Versions are `YY.minor.patch`: the first number is the season the architecture is written for (`27.0.0` opens the 2027 season), the second a feature release within it, the third a fix. Tags carry a `v`.
Robot projects that want to build against a local checkout instead of a release use mainbot's
`local.properties` / `mwlibLocal=true` composite build (see its README).


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

## Documentation

Feature guides live in [`docs/`](docs/):

- [State machines](docs/state-machines.md): `MwSubsystem` and `MwSuperStateMachine`, the robot's super state machine that buttons and autos ask for states.
- [Behavior-tree autos](docs/behavior-tree-autos.md): the `com.marswars.bt` engine, `BehaviorTreeAuto`, the node library, run logs and the live view.
- [Swerve and robot identity](docs/swerve-and-robot-identity.md): `MwSwerveSubsystem`, `SwerveDriveConfig.builder()` and `RobotIdentity`.

The team documentation site has the longer how-to pages: [Robot development](https://frc-team-4143.github.io/docs/software/robot_dev/quick-ref).

## Packages

- `com.marswars.swerve_lib` - Swerve drive implementation (see [Swerve and robot identity](docs/swerve-and-robot-identity.md))
- `com.marswars.mechanisms` - Mechanism base classes
- `com.marswars.geometry` - Geometric utilities
- `com.marswars.logging` - Logging utilities
- `com.marswars.util` - General utilities, including `RobotIdentity`
- `com.marswars.subsystem` - Subsystem state machines and the robot super state machine (see [State machines](docs/state-machines.md))
- `com.marswars.bt` - Behavior-tree engine for autos (see [Behavior-tree autos](docs/behavior-tree-autos.md))
- `com.marswars.auto` - `AutoRoutine`, `AutoManager`, `BehaviorTreeAuto` and the command-based `Auto`

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
- `MwSuperStateMachine` - The robot's super state machine; `RobotContainer` extends it (see [State machines](docs/state-machines.md))
- `SubsystemManager` - Centralized subsystem management

### Mechanisms
- `ArmMech` - Multi-motor arm mechanism with position/velocity control
- `ElevatorMech` - Linear elevator mechanism
- `FlywheelMech` - Velocity-controlled flywheel
- `RollerMech` - Simple roller mechanism

