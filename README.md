# MW-Lib - Mars Wars FRC Library

A Java library for FRC (FIRST Robotics Competition) teams, providing utilities for swerve drive, mechanisms, geometry, and more.

## Features

- **Swerve Drive Library**: Complete swerve drive implementation with Phoenix 6 support
- **Mechanism Classes**: Base classes for arms, elevators, flywheels, and rollers
- **Geometry Utilities**: Regions, splines, and geometric calculations
- **Logging Integration**: Built-in support for elastic logging
- **Proxy Server**: Communication utilities for robot data
- **Constants Loading**: JSON-based configuration management

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

Example usage:

```java
import com.marswars.swerve_lib.SwerveMech;
import com.marswars.mechanisms.ArmMech;
import com.marswars.util.ConstantsLoader;

// Load constants from JSON
ConstantsLoader constants = ConstantsLoader.getInstance();

// Use swerve drive
SwerveMech swerve = new SwerveMech(config);

// Use arm mechanism
ArmMech arm = new ArmMech(config);
```

## Dependencies

This library depends on:
- WPILib 2026+
- Phoenix 6 (CTRE)
- Jackson (JSON processing)
- Various vendor libraries (Maplesim, Playing With Fusion, etc.)

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

