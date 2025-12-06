package frc.mw_lib.swerve_lib;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import dev.doglog.DogLog;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.mw_lib.mechanisms.MechBase;
import frc.mw_lib.swerve_lib.ChassisRequest.ChassisRequestParameters;
import frc.mw_lib.swerve_lib.gyro.Gyro;
import frc.mw_lib.swerve_lib.gyro.GyroPigeon2;
import frc.mw_lib.swerve_lib.module.Module;
import frc.mw_lib.swerve_lib.module.ModuleTalonFX;

public class SwerveMech extends MechBase {

  public SwerveModuleState[] module_states = new SwerveModuleState[] {
      new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };
  public SwerveModulePosition[] module_positions = new SwerveModulePosition[] {
      new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };
  public SwerveModulePosition[] module_deltas = new SwerveModulePosition[] {
      new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };
  public SwerveModulePosition[] last_module_positions = new SwerveModulePosition[] {
      new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };

  public ChassisSpeeds chassis_speeds = new ChassisSpeeds();
  public Rotation2d raw_gyro_rotation = Rotation2d.kZero;
  public Pose2d pose = new Pose2d();

  public ChassisRequest current_request = new ChassisRequest.Idle();
  public ChassisRequestParameters current_request_parameters = new ChassisRequestParameters();

  private final Module[] modules_ = new Module[4]; // FL, FR, BL, BR
  private final Gyro gyro_;

  private final SwerveDriveSimulation swerve_sim_;

  private final SwerveDrivePoseEstimator pose_estimator_;
  private final SwerveDriveKinematics kinematics_;

  public SwerveMech(SwerveDriveConfig config, SwerveDriveSimulation swerve_sim) {
    swerve_sim_ = swerve_sim;
    modules_[0] = new ModuleTalonFX(0, config.FL_MODULE_CONSTANTS, swerve_sim_.getModules()[0]);
    modules_[1] = new ModuleTalonFX(1, config.FR_MODULE_CONSTANTS, swerve_sim_.getModules()[1]);
    modules_[2] = new ModuleTalonFX(2, config.BL_MODULE_CONSTANTS, swerve_sim_.getModules()[2]);
    modules_[3] = new ModuleTalonFX(3, config.BR_MODULE_CONSTANTS, swerve_sim_.getModules()[3]);

    gyro_ = new GyroPigeon2(config.PIGEON2_ID, config.PIGEON2_CANBUS_NAME,
        swerve_sim_.getGyroSimulation());

    // configure the kinematics after the modules are created
    kinematics_ = new SwerveDriveKinematics(getModuleTranslations());

    // Finally configure the Pose Estimator
    pose_estimator_ = new SwerveDrivePoseEstimator(kinematics_, new Rotation2d(), module_positions, Pose2d.kZero);

    // Start odometry thread
    PhoenixOdometryThread.getInstance(config.FL_MODULE_CONSTANTS.drive_motor_config.canbus_name).start();
  }

  @Override
  public void readInputs(double timestamp) {
    for (var module : modules_) {
      module.readInputs(timestamp);
    }
    gyro_.readInputs(timestamp);

    // Update odometry
    double[] sample_timestamps = modules_[0].getOdometryTimestamps(); // All signals are sampled together
    int sample_count = sample_timestamps.length;
    for (int i = 0; i < sample_count; i++) {
      // Read wheel positions and deltas from each module
      for (int module_index = 0; module_index < 4; module_index++) {
        module_positions[module_index] = modules_[module_index].getOdometryPositions()[i];
        module_deltas[module_index] = new SwerveModulePosition(
            module_positions[module_index].distanceMeters
                - last_module_positions[module_index].distanceMeters,
            module_positions[module_index].angle);
        last_module_positions[module_index] = module_positions[module_index];
        module_states[module_index] = modules_[module_index].getState();
      }
      chassis_speeds = kinematics_.toChassisSpeeds(module_states);

      // Update gyro angle
      if (gyro_.isConnected()) {
        // Use the real gyro angle
        raw_gyro_rotation = gyro_.getOdometryYawPositions()[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics_.toTwist2d(module_deltas);
        raw_gyro_rotation = raw_gyro_rotation.plus(new Rotation2d(twist.dtheta));
      }
      pose_estimator_.updateWithTime(sample_timestamps[i], raw_gyro_rotation, module_positions);
    }
    pose = pose_estimator_.getEstimatedPosition();

    if (IS_SIM) {
      DogLog.log(
          "FieldSimulation/RobotPosition", swerve_sim_.getSimulatedDriveTrainPose());
    }
  }

  public void writeOutputs(double timestamp) {
    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules_) {
        module.stop();
      }
    } else {
      current_request_parameters.kinematics = kinematics_;
      current_request_parameters.moduleLocations = getModuleTranslations();
      current_request.apply(current_request_parameters, modules_);
    }
  }

  /** Logs data to DogLog. */
  @Override
  public void logData() {
    DogLog.log(getLoggingKey() + "ModuleStates", module_states);
    DogLog.log(getLoggingKey() + "ModulePositions", module_positions);
    DogLog.log(getLoggingKey() + "ModuleDeltas", module_deltas);
    DogLog.log(getLoggingKey() + "LastModulePositions", last_module_positions);
    DogLog.log(getLoggingKey() + "ChassisSpeeds", chassis_speeds);
    DogLog.log(getLoggingKey() + "RawGyroRotation", raw_gyro_rotation);
    DogLog.log(getLoggingKey() + "Pose", pose);
    DogLog.log(getLoggingKey() + "CurrentRequestType", current_request.getClass().getSimpleName());
  }

  public void resetPose(Pose2d pose) {
    pose_estimator_.resetPosition(pose.getRotation(), new SwerveModulePosition[4], pose);

    if (IS_SIM) {
      swerve_sim_.setSimulationWorldPose(pose);
    }
  }

  public Module[] getModules() {
    return modules_;
  }

  private Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        modules_[0].getTranslation(), // FL
        modules_[1].getTranslation(), // FR
        modules_[2].getTranslation(), // BL
        modules_[3].getTranslation() // BR
    };
  }

}
