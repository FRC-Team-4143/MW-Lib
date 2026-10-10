package com.marswars.swerve_lib;

import static org.junit.jupiter.api.Assertions.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.auto.ChoreoTrajectory;
import com.marswars.mechanisms.MotorConfig;
import com.marswars.swerve_lib.module.ModuleType;
import com.marswars.swerve_lib.module.SwerveModuleConfig;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Exercises the {@link MwSwerveSubsystem} state machine against real (simulated) Phoenix devices.
 * One subsystem is shared by every test because SwerveMech registers its devices with the
 * PhoenixOdometryThread singleton; each test drives it back to IDLE first.
 */
class MwSwerveSubsystemTest {

  private static class TestSwerveConstants extends MwSwerveConstants {
    private final SwerveDriveConfig config =
        new SwerveDriveConfig(
            module(1, 2, 0, 0.3, 0.3),
            module(3, 4, 1, 0.3, -0.3),
            module(5, 6, 2, -0.3, 0.3),
            module(7, 8, 3, -0.3, -0.3),
            0,
            "rio");

    private static SwerveModuleConfig module(
        int drive_id, int steer_id, int encoder_id, double x, double y) {
      SwerveModuleConfig module = new SwerveModuleConfig();
      module.module_type = ModuleType.getModuleType("MK4I-L2");
      module.encoder_type = SwerveModuleConfig.EncoderType.ANALOG_ENCODER;
      module.encoder_id = encoder_id;
      module.wheel_radius_m = 0.05;
      module.speed_at_12_volts = 5.0;
      module.location_x = x;
      module.location_y = y;
      module.drive_motor_config = motor(drive_id);
      module.steer_motor_config = motor(steer_id);
      return module;
    }

    private static MotorConfig motor(int can_id) {
      MotorConfig motor = new MotorConfig();
      motor.can_id = can_id;
      motor.apply(new TalonFXConfiguration());
      return motor;
    }

    @Override
    public SwerveDriveConfig getDriveConfig() {
      return config;
    }
  }

  private static class TestSwerveSubsystem extends MwSwerveSubsystem<TestSwerveConstants> {
    TestSwerveSubsystem() {
      super(
          new TestSwerveConstants(),
          () -> pose,
          new SwerveDriverInputs(() -> 0.0, () -> 0.0, () -> 0.0, () -> pov));
    }
  }

  private static Pose2d pose = Pose2d.kZero;
  private static Optional<Rotation2d> pov = Optional.empty();
  private static TestSwerveSubsystem swerve;

  @BeforeAll
  static void setUp() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    swerve = new TestSwerveSubsystem();
  }

  @AfterAll
  static void tearDown() {
    SimHooks.resumeTiming();
  }

  @BeforeEach
  void resetToIdle() {
    pose = Pose2d.kZero;
    pov = Optional.empty();
    tick(SwerveStates.IDLE);
  }

  private static void tick(SwerveStates wanted) {
    swerve.setWantedState(wanted);
    swerve.update(0.0);
  }

  /** A stationary 2 s trajectory at the origin, so the look-ahead pause never triggers. */
  private static ChoreoTrajectory stationaryTrajectory() {
    var samples =
        List.of(
            new SwerveSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[4], new double[4]),
            new SwerveSample(2, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[4], new double[4]));
    return new ChoreoTrajectory(new Trajectory<>("still", samples, List.of(), List.of()), false);
  }

  @Test
  void crawlVariantPreservesFrameAndRotationLock() {
    assertEquals(
        SwerveStates.CRAWL_FIELD_CENTRIC,
        MwSwerveSubsystem.crawlVariantOf(SwerveStates.FIELD_CENTRIC));
    assertEquals(
        SwerveStates.CRAWL_FIELD_CENTRIC_ROTATION_LOCK,
        MwSwerveSubsystem.crawlVariantOf(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK));
    assertEquals(
        SwerveStates.CRAWL_ROBOT_CENTRIC_ROTATION_LOCK,
        MwSwerveSubsystem.crawlVariantOf(SwerveStates.CHASSIS_SPEEDS_ROTATION_LOCK));
    assertEquals(
        SwerveStates.CRAWL_ROBOT_CENTRIC,
        MwSwerveSubsystem.crawlVariantOf(SwerveStates.CHOREO_PATH));
  }

  @Test
  void holdingPovForcesCrawl() {
    pov = Optional.of(Rotation2d.kCCW_Pi_2);
    tick(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
    assertEquals(SwerveStates.CRAWL_FIELD_CENTRIC_ROTATION_LOCK, swerve.getSystemState());

    pov = Optional.empty();
    tick(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
    assertEquals(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK, swerve.getSystemState());
  }

  @Test
  void idleCommandsZeroSpeeds() {
    swerve.setDesiredChassisSpeed(new edu.wpi.first.math.kinematics.ChassisSpeeds(1, 1, 1));
    tick(SwerveStates.IDLE);
    var speeds = swerve.getDesiredChassisSpeeds();
    assertEquals(0.0, speeds.vxMetersPerSecond, 1e-9);
    assertEquals(0.0, speeds.vyMetersPerSecond, 1e-9);
    assertEquals(0.0, speeds.omegaRadiansPerSecond, 1e-9);
  }

  @Test
  void choreoTimerRestartsOnlyWhenEnteringFromNonChoreoState() {
    swerve.setDesiredChoreoTrajectory(stationaryTrajectory());
    tick(SwerveStates.CHOREO_PATH);
    SimHooks.stepTiming(0.5);
    tick(SwerveStates.CHOREO_PATH);
    assertTrue(swerve.hasChoreoTimeElapsed(0.4));

    // Switching between the two Choreo states keeps the timer running
    tick(SwerveStates.CHOREO_PATH_ROTATION_LOCK);
    assertEquals(SwerveStates.CHOREO_PATH_ROTATION_LOCK, swerve.getSystemState());
    assertTrue(swerve.hasChoreoTimeElapsed(0.4));

    // Leaving and re-entering restarts it
    tick(SwerveStates.IDLE);
    assertFalse(swerve.hasChoreoTimeElapsed(0.4));
    tick(SwerveStates.CHOREO_PATH);
    assertFalse(swerve.hasChoreoTimeElapsed(0.4));
  }

  @Test
  void tractorBeamDrivesTowardTarget() {
    // Default tractor beam gains are zero, so the output is the static-friction term alone
    swerve.setDesiredTractorBeamPose(new Pose2d(0.0, 2.0, Rotation2d.kZero));
    tick(SwerveStates.TRACTOR_BEAM);
    var speeds = swerve.getDesiredChassisSpeeds();
    TestSwerveConstants constants = new TestSwerveConstants();
    double expected =
        constants.TRACTOR_BEAM_STATIC_FRICTION_CONSTANT * constants.MAX_TRANSLATION_RATE;
    assertEquals(0.0, speeds.vxMetersPerSecond, 1e-9);
    assertEquals(expected, speeds.vyMetersPerSecond, 1e-9);
    assertFalse(swerve.isAtTractorBeamSetpoint());

    pose = new Pose2d(0.0, 2.0, Rotation2d.kZero);
    assertTrue(swerve.isAtTractorBeamSetpoint());
  }
}
