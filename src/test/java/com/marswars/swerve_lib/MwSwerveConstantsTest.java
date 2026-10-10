package com.marswars.swerve_lib;

import static org.junit.jupiter.api.Assertions.*;

import com.marswars.swerve_lib.module.ModuleType;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

/** The robot-constants pattern: base fields, variants reassign them in their constructors. */
class MwSwerveConstantsTest {

  private static class BaseConstants extends MwSwerveConstants {
    public double WHEEL_RADIUS_METERS = 0.05;

    @Override
    public SwerveDriveConfig getDriveConfig() {
      return SwerveDriveConfig.builder()
          .moduleType(ModuleType.getModuleType("MK4I-L2"))
          .wheelRadius(WHEEL_RADIUS_METERS)
          .speedAt12V(5.0)
          .frontLeft(1, 2, 0, new Translation2d(0.3, 0.3))
          .frontRight(3, 4, 1, new Translation2d(0.3, -0.3))
          .backLeft(5, 6, 2, new Translation2d(-0.3, 0.3))
          .backRight(7, 8, 3, new Translation2d(-0.3, -0.3))
          .build();
    }
  }

  private static class VariantConstants extends BaseConstants {
    VariantConstants() {
      WHEEL_RADIUS_METERS = 0.06;
      MAX_TRANSLATION_RATE = 4.2;
    }
  }

  @Test
  void baseUsesLibraryDefaults() {
    var constants = new BaseConstants();
    assertEquals(5.0, constants.MAX_TRANSLATION_RATE, 1e-9);
    assertEquals(0.05, constants.getDriveConfig().FL_MODULE_CONSTANTS.wheel_radius_m, 1e-9);
  }

  @Test
  void variantConstructorValuesReachTunablesAndDriveConfig() {
    var constants = new VariantConstants();
    assertEquals(4.2, constants.MAX_TRANSLATION_RATE, 1e-9);
    assertEquals(0.06, constants.getDriveConfig().BR_MODULE_CONSTANTS.wheel_radius_m, 1e-9);
    assertEquals(40.0, constants.MAX_TRANSLATION_ACCEL, 1e-9); // untouched default
  }
}
