package frc.mw_lib.swerve_lib;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.ctre.phoenix6.configs.SlotConfigs;

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
import edu.wpi.first.wpilibj.Timer;
import frc.mw_lib.mechanisms.MechBase;
import frc.mw_lib.swerve_lib.ChassisRequest.ChassisRequestParameters;
import frc.mw_lib.swerve_lib.gyro.Gyro;
import frc.mw_lib.swerve_lib.gyro.GyroPigeon2;
import frc.mw_lib.swerve_lib.module.Module;
import frc.mw_lib.swerve_lib.module.ModuleTalonFX;
import frc.mw_lib.util.TunablePid;

public class SwerveMech extends MechBase {

  private SwerveModuleState[] module_states = new SwerveModuleState[] {
      new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };
  private SwerveModulePosition[] module_positions = new SwerveModulePosition[] {
      new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };
  private SwerveModulePosition[] module_deltas = new SwerveModulePosition[] {
      new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };
  private SwerveModulePosition[] last_module_positions = new SwerveModulePosition[] {
      new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };

  private ChassisSpeeds chassis_speeds = new ChassisSpeeds();
  private Rotation2d raw_gyro_rotation = Rotation2d.kZero;
  private Pose2d pose = new Pose2d();

  private ChassisRequest current_request = new ChassisRequest.Idle();
  private ChassisRequestParameters current_request_parameters = new ChassisRequestParameters();

  private final Module[] modules_ = new Module[4]; // FL, FR, BL, BR
  private final Gyro gyro_;

  private final SwerveDriveSimulation swerve_sim_;

  private final SwerveDrivePoseEstimator pose_estimator_;
  private final SwerveDriveKinematics kinematics_;

  public SwerveMech(String logging_prefix, SwerveDriveConfig config, DriveTrainSimulationConfig sim_config) {
    super(logging_prefix);

    swerve_sim_ = new SwerveDriveSimulation(sim_config, Pose2d.kZero);

    modules_[0] = new ModuleTalonFX(logging_prefix, 0, config.FL_MODULE_CONSTANTS, swerve_sim_.getModules()[0]);
    modules_[1] = new ModuleTalonFX(logging_prefix, 1, config.FR_MODULE_CONSTANTS, swerve_sim_.getModules()[1]);
    modules_[2] = new ModuleTalonFX(logging_prefix, 2, config.BL_MODULE_CONSTANTS, swerve_sim_.getModules()[2]);
    modules_[3] = new ModuleTalonFX(logging_prefix, 3, config.BR_MODULE_CONSTANTS, swerve_sim_.getModules()[3]);

    gyro_ = new GyroPigeon2(logging_prefix, config.PIGEON2_ID, config.PIGEON2_CANBUS_NAME,
        swerve_sim_.getGyroSimulation());

    // configure the kinematics after the modules are created
    kinematics_ = new SwerveDriveKinematics(getModuleTranslations());

    // Finally configure the Pose Estimator
    pose_estimator_ = new SwerveDrivePoseEstimator(kinematics_, new Rotation2d(), module_positions, Pose2d.kZero);

    // Start odometry thread
    PhoenixOdometryThread.configure(config.FL_MODULE_CONSTANTS.drive_motor_config.canbus_name,
        config.FL_MODULE_CONSTANTS.wheel_radius_m);
    PhoenixOdometryThread.getInstance().start();

    // TODO: Load the default gains from config
    TunablePid.create(getLoggingKey() + "Drive/PositionGains", this::setDrivePositionGains, new SlotConfigs());
    TunablePid.create(getLoggingKey() + "Drive/VelocityGains", this::setDriveVelocityGains, new SlotConfigs());
    TunablePid.create(getLoggingKey() + "Turn/PositionGains", this::setSteerGains, new SlotConfigs());
  }

  @Override
  public void readInputs(double timestamp) {
    for (var module : modules_) {
      module.readInputs(timestamp);
    }
    gyro_.readInputs(timestamp);

    for(int i = 0; i < modules_.length; i++) {
      module_states[i] = modules_[i].getState();
      module_positions[i] = modules_[i].getPosition();
      module_deltas[i] = new SwerveModulePosition(module_positions[i].distanceMeters - last_module_positions[i].distanceMeters,
          module_positions[i].angle);
      last_module_positions[i] = module_positions[i];
    }
    chassis_speeds = kinematics_.toChassisSpeeds(module_states);

    // Update gyro angle
    if (gyro_.isConnected()) {
      // Use the real gyro angle
      raw_gyro_rotation = gyro_.getYawPosition();
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics_.toTwist2d(module_deltas);
      raw_gyro_rotation = raw_gyro_rotation.plus(new Rotation2d(twist.dtheta));
    }

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

  /**
   * Sets the chassis request for the swerve drive.
   * @param request
   */
  public void setChassisRequest(ChassisRequest request) {
    current_request = request;
  }

  /**
   * Sets the chassis request parameters for the swerve drive.
   * @param pose The current robot pose.
   * @param operator_forward_direction The operator's forward direction.
   */
  public void setChassisRequestParameters(Pose2d pose,
      Rotation2d operator_forward_direction) {
    current_request_parameters.currentChassisSpeed = chassis_speeds;
    current_request_parameters.currentPose = pose;
    current_request_parameters.updatePeriod = Timer.getFPGATimestamp() - current_request_parameters.timestamp;
    current_request_parameters.timestamp = Timer.getFPGATimestamp();
    current_request_parameters.operatorForwardDirection = operator_forward_direction;
  }

  /**
   * Resets the robot pose in the pose estimator.
   * @param pose The new robot pose.
   */
  public void resetPose(Pose2d pose) {
    pose_estimator_.resetPosition(pose.getRotation(), new SwerveModulePosition[4], pose);

    if (IS_SIM) {
      swerve_sim_.setSimulationWorldPose(pose);
    }
  }

  /** Returns the current estimated robot pose. */
  public Module[] getModules() {
    return modules_;
  }

  /**
   * Returns the translations of the swerve drive modules.
   * @return Translation2d[] array of module translations
   */
  private Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        modules_[0].getTranslation(), // FL
        modules_[1].getTranslation(), // FR
        modules_[2].getTranslation(), // BL
        modules_[3].getTranslation() // BR
    };
  }

  /**
   * Returns the measured chassis speeds of the robot.
   * @return ChassisSpeeds object representing the robot's chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return chassis_speeds;
  }

  /**
   * Returns the module states of the swerve drive.
   * @return SwerveModuleState[] array of module states
   */
  public SwerveModuleState[] getModuleStates() {
    return module_states;
  }

  /**
   * Returns the module positions of the swerve drive.
   * @return SwerveModulePosition[] array of module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    return module_positions;
  }

  /**
   * Returns the raw gyro rotation used for odometry.
   * @return Rotation2d representing the raw gyro rotation
   */
  public Rotation2d getRawGyroRotation() {
    return raw_gyro_rotation;
  }

  /**
   * Returns the current estimated robot pose. Purely from odometry.
   * @return Pose2d representing the robot's estimated pose
   */
  public Pose2d getPose() {
    return pose;
  }

  /**
   * Returns the swerve drive simulation instance.
   * @return SwerveDriveSimulation object representing the swerve drive simulation
   */
  public SwerveDriveSimulation getSwerveSimulation() {
    return swerve_sim_;
  }

  private void setDrivePositionGains(SlotConfigs gains){
    setDriveGains(0, gains);
  }

  private void setDriveVelocityGains(SlotConfigs gains){
    setDriveGains(1, gains);
  }

  private void setDriveGains(int slot, SlotConfigs gains){
    for (var module : modules_) {
      module.setDriveGains(slot, gains);
    }
  }

  private void setSteerGains(SlotConfigs gains){
    for (var module : modules_) {
      module.setSteerGains(gains);
    }
  }

}
