# Swerve and robot identity

`MwSwerveSubsystem` is the library's swerve drive: a robot extends it with its own constants, a pose supplier and the driver's joystick inputs. `RobotIdentity` resolves which physical robot the code is running on, so those constants can have per-robot variants.

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
