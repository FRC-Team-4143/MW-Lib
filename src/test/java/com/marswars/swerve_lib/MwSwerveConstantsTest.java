package com.marswars.swerve_lib;

import static org.junit.jupiter.api.Assertions.*;

import com.marswars.swerve_lib.module.ModuleType;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class MwSwerveConstantsTest {

  private static SwerveDriveConfig drive() {
    return SwerveDriveConfig.builder()
        .moduleType(ModuleType.getModuleType("MK4I-L2"))
        .wheelRadius(0.05)
        .speedAt12V(5.0)
        .frontLeft(1, 2, 0, new Translation2d(0.3, 0.3))
        .frontRight(3, 4, 1, new Translation2d(0.3, -0.3))
        .backLeft(5, 6, 2, new Translation2d(-0.3, 0.3))
        .backRight(7, 8, 3, new Translation2d(-0.3, -0.3))
        .build();
  }

  private static class DefaultConstants extends MwSwerveConstants {
    DefaultConstants(SwerveDriveConfig drive) {
      super(drive);
    }
  }

  private static class TunedConstants extends MwSwerveConstants {
    TunedConstants(SwerveDriveConfig drive) {
      super(
          drive,
          new Tuning().maxTranslationRate(4.2).choreoThetaGains(9, 0.5, 0.25).choreoLookAhead(0.75));
    }
  }

  @Test
  void defaultsApplyWithoutTuning() {
    SwerveDriveConfig drive = drive();
    var constants = new DefaultConstants(drive);
    assertSame(drive, constants.DRIVE_CONFIG);
    assertEquals(5.0, constants.MAX_TRANSLATION_RATE, 1e-9);
    assertEquals(10.0, constants.HEADING_CONTROLLER_KP, 1e-9);
    assertEquals(12.0, constants.CHOREO_THETA_CONTROLLER_KP, 1e-9);
    assertEquals(1.0, constants.CHOREO_LOOK_AHEAD, 1e-9);
  }

  @Test
  void tuningOverridesOnlyWhatIsSet() {
    var constants = new TunedConstants(drive());
    assertEquals(4.2, constants.MAX_TRANSLATION_RATE, 1e-9);
    assertEquals(9.0, constants.CHOREO_THETA_CONTROLLER_KP, 1e-9);
    assertEquals(0.5, constants.CHOREO_THETA_CONTROLLER_KI, 1e-9);
    assertEquals(0.25, constants.CHOREO_THETA_CONTROLLER_KD, 1e-9);
    assertEquals(0.75, constants.CHOREO_LOOK_AHEAD, 1e-9);
    // untouched values keep their defaults
    assertEquals(40.0, constants.MAX_TRANSLATION_ACCEL, 1e-9);
    assertEquals(7.0, constants.CHOREO_TRANSLATION_CONTROLLER_KP, 1e-9);
  }
}
