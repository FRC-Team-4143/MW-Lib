package frc.mw_lib.mechanisms;

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
import frc.mw_lib.swerve_lib.ChassisRequest;
import frc.mw_lib.swerve_lib.ChassisRequest.ChassisRequestParameters;
import frc.mw_lib.swerve_lib.gyro.Gyro;
import frc.mw_lib.swerve_lib.gyro.GyroIOPigeon2;
import frc.mw_lib.swerve_lib.gyro.GyroIOSim;
import frc.mw_lib.swerve_lib.module.Module;
import frc.mw_lib.swerve_lib.module.ModuleIOTalonFXReal;
import frc.mw_lib.swerve_lib.module.ModuleIOTalonFXSim;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveConstants;

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

  private final SwerveDrivePoseEstimator pose_estimator_;
  private final SwerveDriveKinematics kinematics_;

  public SwerveMech(SwerveConstants constants) {
    // configure the kinematics after the modules are created
    kinematics_ = new SwerveDriveKinematics(getModuleTranslations());

    // Finally configure the Pose Estimator
    pose_estimator_ = new SwerveDrivePoseEstimator(
        kinematics_, new Rotation2d(), module_positions, Pose2d.kZero);

    if (IS_SIM) {
      gyro_ = new Gyro(new GyroIOSim(Constants.SWERVE_SIMULATOR.getGyroSimulation()));
      // Configure Modules
      modules_[0] = new Module(
          new ModuleIOTalonFXSim(
              constants.SIM_FL_MODULE_CONSTANTS, constants.MODULE_CANBUS_NAME,
              Constants.SWERVE_SIMULATOR.getModules()[0]),
          0,
          constants.SIM_FL_MODULE_CONSTANTS);
      modules_[1] = new Module(
          new ModuleIOTalonFXSim(
              constants.SIM_FR_MODULE_CONSTANTS, constants.MODULE_CANBUS_NAME,
              Constants.SWERVE_SIMULATOR.getModules()[1]),
          1,
          constants.SIM_FR_MODULE_CONSTANTS);
      modules_[2] = new Module(
          new ModuleIOTalonFXSim(
              constants.SIM_BL_MODULE_CONSTANTS, constants.MODULE_CANBUS_NAME,
              Constants.SWERVE_SIMULATOR.getModules()[2]),
          2,
          constants.SIM_BL_MODULE_CONSTANTS);
      modules_[3] = new Module(
          new ModuleIOTalonFXSim(
              constants.SIM_BR_MODULE_CONSTANTS, constants.MODULE_CANBUS_NAME,
              Constants.SWERVE_SIMULATOR.getModules()[3]),
          3,
          constants.SIM_BR_MODULE_CONSTANTS);
    } else {
      gyro_ = new Gyro(new GyroIOPigeon2(constants.PIGEON2_ID, constants.PIGEON2_CANBUS_NAME));
      // Configure Modules
      modules_[0] = new Module(
          new ModuleIOTalonFXReal(constants.FL_MODULE_CONSTANTS, constants.MODULE_CANBUS_NAME),
          0,
          constants.FL_MODULE_CONSTANTS);
      modules_[1] = new Module(
          new ModuleIOTalonFXReal(constants.FR_MODULE_CONSTANTS, constants.MODULE_CANBUS_NAME),
          1,
          constants.FR_MODULE_CONSTANTS);
      modules_[2] = new Module(
          new ModuleIOTalonFXReal(constants.BL_MODULE_CONSTANTS, constants.MODULE_CANBUS_NAME),
          2,
          constants.BL_MODULE_CONSTANTS);
      modules_[3] = new Module(
          new ModuleIOTalonFXReal(constants.BR_MODULE_CONSTANTS, constants.MODULE_CANBUS_NAME),
          3,
          constants.BR_MODULE_CONSTANTS);
    }

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();
  }

  @Override
  public void readInputs(double timestamp) {
    PhoenixOdometryThread.getInstance().getOdometryLock().lock(); // Prevents odometry updates while reading data
    for (var module : modules_) {
      module.periodic();
    }
    gyro_.periodic();
    PhoenixOdometryThread.getInstance().getOdometryLock().unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules_) {
        module.stop();
      }
    }

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
          "FieldSimulation/RobotPosition", Constants.SWERVE_SIMULATOR.getSimulatedDriveTrainPose());
    }
  }

  public void writeOutputs(double timestamp) {
    current_request_parameters.kinematics = kinematics_;
    current_request_parameters.moduleLocations = getModuleTranslations();
    current_request.apply(current_request_parameters, modules_);
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
      Constants.SWERVE_SIMULATOR.setSimulationWorldPose(pose);
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
