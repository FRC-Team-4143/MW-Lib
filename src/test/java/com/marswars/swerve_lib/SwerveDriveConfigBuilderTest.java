package com.marswars.swerve_lib;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.swerve_lib.module.ModuleType;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class SwerveDriveConfigBuilderTest {

  private static SwerveDriveConfig.Builder base(String module_type) {
    return SwerveDriveConfig.builder()
        .canbus("CANivore")
        .pigeon2Id(5)
        .moduleType(ModuleType.getModuleType(module_type))
        .wheelRadius(0.05)
        .speedAt12V(4.5)
        .driveGains(new Slot0Configs(), new Slot1Configs().withKV(0.1))
        .driveCurrentLimits(35, 105)
        .steerMotor(TalonMotorType.X44)
        .steerGains(new Slot0Configs().withKP(80), new Slot1Configs())
        .steerCurrentLimits(20, 60)
        .frontLeft(1, 2, 0, new Translation2d(0.2, 0.2))
        .frontRight(3, 4, 1, new Translation2d(0.2, -0.2))
        .backLeft(5, 6, 2, new Translation2d(-0.2, 0.2))
        .backRight(7, 8, 3, new Translation2d(-0.2, -0.2));
  }

  @Test
  void wiresModulesAndConventions() {
    SwerveDriveConfig config = base("TSN-P13-S18").build();

    assertEquals(5, config.PIGEON2_ID);
    assertEquals("CANivore", config.PIGEON2_CANBUS_NAME);

    var fr = config.FR_MODULE_CONSTANTS;
    assertEquals(3, fr.drive_motor_config.can_id);
    assertEquals(4, fr.steer_motor_config.can_id);
    assertEquals(1, fr.encoder_id);
    assertEquals(-0.2, fr.location_y, 1e-9);
    assertEquals("CANivore", fr.steer_motor_config.canbus_name);
    assertEquals(TalonMotorType.X60, fr.drive_motor_config.motor_type);
    assertEquals(TalonMotorType.X44, fr.steer_motor_config.motor_type);

    var drive = fr.drive_motor_config.getAsFXConfig();
    assertEquals(NeutralModeValue.Brake, drive.MotorOutput.NeutralMode);
    assertEquals(InvertedValue.CounterClockwise_Positive, drive.MotorOutput.Inverted);
    assertEquals(35, drive.CurrentLimits.SupplyCurrentLimit, 1e-9);
    assertEquals(105, drive.CurrentLimits.StatorCurrentLimit, 1e-9);
    assertTrue(drive.CurrentLimits.SupplyCurrentLimitEnable);
    assertEquals(0.1, drive.Slot1.kV, 1e-9);
    assertEquals(80, fr.steer_motor_config.getAsFXConfig().Slot0.kP, 1e-9);
    assertEquals(60, fr.steer_motor_config.getAsFXConfig().CurrentLimits.StatorCurrentLimit, 1e-9);
  }

  @Test
  void steerInversionComesFromModuleType() {
    var tsn = base("TSN-P13-S18").build().FL_MODULE_CONSTANTS.steer_motor_config.getAsFXConfig();
    var mk4i = base("MK4I-L2").build().FL_MODULE_CONSTANTS.steer_motor_config.getAsFXConfig();
    assertEquals(InvertedValue.CounterClockwise_Positive, tsn.MotorOutput.Inverted);
    assertEquals(InvertedValue.Clockwise_Positive, mk4i.MotorOutput.Inverted);
  }

  @Test
  void everyMotorGetsItsOwnConfiguration() {
    // ModuleTalonFX mutates the motor configuration, so modules must not share one
    SwerveDriveConfig config = base("MK4I-L2").build();
    assertNotSame(
        config.FL_MODULE_CONSTANTS.drive_motor_config.getAsFXConfig(),
        config.FR_MODULE_CONSTANTS.drive_motor_config.getAsFXConfig());
    assertNotSame(
        config.FL_MODULE_CONSTANTS.drive_motor_config.getAsFXConfig(),
        config.FL_MODULE_CONSTANTS.steer_motor_config.getAsFXConfig());
  }

  @Test
  void perModuleDriveInversion() {
    SwerveDriveConfig config =
        base("MK4I-L2").module(1, 3, 4, 1, new Translation2d(0.2, -0.2), true).build();
    assertEquals(
        InvertedValue.Clockwise_Positive,
        config.FR_MODULE_CONSTANTS.drive_motor_config.getAsFXConfig().MotorOutput.Inverted);
    assertEquals(
        InvertedValue.CounterClockwise_Positive,
        config.FL_MODULE_CONSTANTS.drive_motor_config.getAsFXConfig().MotorOutput.Inverted);
  }

  @Test
  void rejectsMissingRequiredValues() {
    assertThrows(IllegalStateException.class, () -> SwerveDriveConfig.builder().build());
    var missing_module =
        SwerveDriveConfig.builder()
            .moduleType(ModuleType.getModuleType("MK4I-L2"))
            .wheelRadius(0.05)
            .speedAt12V(4.5)
            .frontLeft(1, 2, 0, Translation2d.kZero);
    var e = assertThrows(IllegalStateException.class, missing_module::build);
    assertTrue(e.getMessage().contains("module 1"), e.getMessage());
  }
}
