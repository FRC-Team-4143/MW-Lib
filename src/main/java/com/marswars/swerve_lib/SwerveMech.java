package com.marswars.swerve_lib;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.marswars.mechanisms.MechBase;
import com.marswars.sensors.gyro.Gyro;
import com.marswars.sensors.gyro.Pigeon2Gyro;
import com.marswars.swerve_lib.ChassisRequest.ChassisRequestParameters;
import com.marswars.swerve_lib.module.Module;
import com.marswars.swerve_lib.module.ModuleTalonFX;
import com.marswars.util.TunablePid;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class SwerveMech extends MechBase {

    private SwerveModuleState[] current_module_states =
            new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            };
    private SwerveModuleState[] setpoint_module_states =
            new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            };
    private SwerveModulePosition[] module_positions =
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private SwerveModulePosition[] module_deltas =
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private SwerveModulePosition[] last_module_positions =
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };

    private ChassisSpeeds chassis_speeds = new ChassisSpeeds();
    private Rotation2d raw_gyro_rotation = Rotation2d.kZero;

    private ChassisRequest current_request = new ChassisRequest.Idle();
    private ChassisRequestParameters current_request_parameters = new ChassisRequestParameters();

    private final Module[] modules_ = new Module[4]; // FL, FR, BL, BR
    private final Gyro gyro_;

    private final SwerveDriveSimulation swerve_sim_;

    private final SwerveDriveKinematics kinematics_;

    private final Trigger user_button_trigger_ = new Trigger(RobotController::getUserButton);
    private final Trigger ds_enabled_trigger_ = new Trigger(DriverStation::isEnabled);

    public SwerveMech(
            String logging_prefix,
            SwerveDriveConfig config,
            DriveTrainSimulationConfig sim_config) {
        super(logging_prefix);

        // Configure the odom thread
        PhoenixOdometryThread.configure(
                config.FL_MODULE_CONSTANTS.drive_motor_config.canbus_name,
                config.FL_MODULE_CONSTANTS.wheel_radius_m);

        swerve_sim_ = new SwerveDriveSimulation(sim_config, Pose2d.kZero);

        modules_[0] =
                new ModuleTalonFX(
                        logging_prefix, 0, config.FL_MODULE_CONSTANTS, swerve_sim_.getModules()[0]);
        modules_[1] =
                new ModuleTalonFX(
                        logging_prefix, 1, config.FR_MODULE_CONSTANTS, swerve_sim_.getModules()[1]);
        modules_[2] =
                new ModuleTalonFX(
                        logging_prefix, 2, config.BL_MODULE_CONSTANTS, swerve_sim_.getModules()[2]);
        modules_[3] =
                new ModuleTalonFX(
                        logging_prefix, 3, config.BR_MODULE_CONSTANTS, swerve_sim_.getModules()[3]);

        gyro_ =
                new Pigeon2Gyro(
                        logging_prefix,
                        config.PIGEON2_ID,
                        config.PIGEON2_CANBUS_NAME,
                        swerve_sim_.getGyroSimulation());

        // configure the kinematics after the modules are created
        kinematics_ = new SwerveDriveKinematics(getModuleTranslations());

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // TODO: Load the default gains from config
        TunablePid.create(
                getLoggingKey() + "Drive/PositionGains",
                this::setDrivePositionGains,
                SlotConfigs.from(config.FL_MODULE_CONSTANTS.drive_motor_config.config.Slot0));
        TunablePid.create(
                getLoggingKey() + "Drive/VelocityGains",
                this::setDriveVelocityGains,
                SlotConfigs.from(config.FL_MODULE_CONSTANTS.drive_motor_config.config.Slot1));
        TunablePid.create(
                getLoggingKey() + "Steer/PositionGains",
                this::setSteerGains,
                SlotConfigs.from(config.FL_MODULE_CONSTANTS.steer_motor_config.config.Slot0));

        user_button_trigger_.onTrue(Commands.runOnce(() -> setNeutralMode(NeutralModeValue.Coast)).ignoringDisable(true));
        ds_enabled_trigger_.onTrue(Commands.runOnce(() -> setNeutralMode(NeutralModeValue.Brake)).ignoringDisable(true));
    }

    @Override
    public void readInputs(double timestamp) {
        for (var module : modules_) {
            module.readInputs(timestamp);
            module.logData();
        }
        gyro_.readInputs(timestamp);
        gyro_.logData();

        for (int i = 0; i < modules_.length; i++) {
            current_module_states[i] = modules_[i].getCurrentState();
            setpoint_module_states[i] = modules_[i].getSetpointState();
            module_positions[i] = modules_[i].getPosition();
            module_deltas[i] =
                    new SwerveModulePosition(
                            module_positions[i].distanceMeters
                                    - last_module_positions[i].distanceMeters,
                            module_positions[i].angle);
            last_module_positions[i] = module_positions[i];
        }
        chassis_speeds = kinematics_.toChassisSpeeds(current_module_states);

        // Update gyro angle
        if (gyro_.isConnected()) {
            // Use the real gyro angle
            raw_gyro_rotation = gyro_.getYawPosition();
        } else {
            // Use the angle delta from the kinematics and module deltas
            Twist2d twist = kinematics_.toTwist2d(module_deltas);
            raw_gyro_rotation = raw_gyro_rotation.plus(new Rotation2d(twist.dtheta));
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

            // Modules do not have a write outputs method
            // for (Module module : modules_) {
            //     module.writeOutputs(timestamp);
            // }
        }
    }

    /** Logs data to DogLog. */
    @Override
    public void logData() {
        DogLog.log(getLoggingKey() + "CurrentModuleStates", current_module_states);
        DogLog.log(getLoggingKey() + "SetpointModuleStates", setpoint_module_states);
        DogLog.log(getLoggingKey() + "ModulePositions", module_positions);
        DogLog.log(getLoggingKey() + "ModuleDeltas", module_deltas);
        DogLog.log(getLoggingKey() + "LastModulePositions", last_module_positions);
        DogLog.log(getLoggingKey() + "ChassisSpeeds", chassis_speeds);
        DogLog.log(getLoggingKey() + "RawGyroRotation", raw_gyro_rotation);
        DogLog.log(
                getLoggingKey() + "CurrentRequestType", current_request.getClass().getSimpleName());
    }

    /**
     * Sets the chassis request for the swerve drive.
     *
     * @param request
     */
    public void setChassisRequest(ChassisRequest request) {
        current_request = request;
    }

    /**
     * Sets the chassis request parameters for the swerve drive.
     *
     * @param pose The current robot pose.
     * @param operator_forward_direction The operator's forward direction.
     */
    public void setChassisRequestParameters(Pose2d pose, Rotation2d operator_forward_direction) {
        current_request_parameters.currentChassisSpeed = chassis_speeds;
        current_request_parameters.currentPose = pose;
        current_request_parameters.updatePeriod =
                Timer.getFPGATimestamp() - current_request_parameters.timestamp;
        current_request_parameters.timestamp = Timer.getFPGATimestamp();
        current_request_parameters.operatorForwardDirection = operator_forward_direction;
    }

    /**
     * Sets the gyro yaw position.
     *
     * @param yaw The desired yaw rotation.
     */
    public void setGyro(Rotation2d yaw) {
        gyro_.setYaw(yaw);
    }

    /** Stores the current encoder readings as offsets */
    public void setModuleOffsets() {
        for (var module : modules_) {
            module.setModuleOffset();
        }
    }

    public void setNeutralMode(NeutralModeValue mode) {
        for (var module : modules_) {
            module.setNeutralMode(mode);
        }
    }

    /** Returns the current estimated robot pose. */
    public Module[] getModules() {
        return modules_;
    }

    /**
     * Returns the translations of the swerve drive modules.
     *
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
     *
     * @return ChassisSpeeds object representing the robot's chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return chassis_speeds;
    }

    /**
     * Returns the module states of the swerve drive.
     *
     * @return SwerveModuleState[] array of module states
     */
    public SwerveModuleState[] getModuleStates() {
        return current_module_states;
    }

    /**
     * Returns the module positions of the swerve drive.
     *
     * @return SwerveModulePosition[] array of module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return module_positions;
    }

    /**
     * Returns the raw gyro rotation used for odometry.
     *
     * @return Rotation2d representing the raw gyro rotation
     */
    public Rotation2d getRawGyroRotation() {
        return raw_gyro_rotation;
    }

    /**
     * Returns the swerve drive simulation instance.
     *
     * @return SwerveDriveSimulation object representing the swerve drive simulation
     */
    public SwerveDriveSimulation getSwerveSimulation() {
        return swerve_sim_;
    }

    /**
     * Applies simple direct pose control to the robot in simulation based on joystick inputs.
     * This bypasses all swerve drive mathematics, directly updates the robot's pose,
     * and fakes the chassis speeds to match the movement for realistic feedback.
     * This method should only be called in simulation mode.
     *
     * @param joystick_twist Twist2d representing the desired movement (dx, dy, dtheta)
     * @param translation_scale Scale factor for translation movement (meters per cycle)
     * @param rotation_scale Scale factor for rotational movement (radians per cycle)
     */
    public void applySimpleSimulationControl(Twist2d joystick_twist, double translation_scale, double rotation_scale) {
        if (swerve_sim_ != null) {
            Pose2d current_pose = swerve_sim_.getSimulatedDriveTrainPose();
            
            // Calculate movement based on joystick inputs
            double delta_x = joystick_twist.dx * translation_scale;
            double delta_y = joystick_twist.dy * translation_scale;
            double delta_theta = joystick_twist.dtheta * rotation_scale;
            
            // Calculate new pose
            double new_x = current_pose.getX() + delta_x;
            double new_y = current_pose.getY() + delta_y;
            double new_rotation = current_pose.getRotation().getRadians() + delta_theta;
            
            Pose2d new_pose = new Pose2d(new_x, new_y, new Rotation2d(new_rotation));
            
            // Set the new pose directly in simulation
            swerve_sim_.setSimulationWorldPose(new_pose);
            
            // Update the gyro rotation for odometry
            raw_gyro_rotation = new_pose.getRotation();
            
            // Always fake the chassis speeds to match the joystick inputs
            double loop_time = 0.02; // seconds (50 Hz)
            
            // Calculate chassis speeds based on actual movement per cycle
            ChassisSpeeds target_speeds = new ChassisSpeeds(
                (joystick_twist.dx * translation_scale) / loop_time,  // vx in m/s
                (joystick_twist.dy * translation_scale) / loop_time,  // vy in m/s
                (joystick_twist.dtheta * rotation_scale) / loop_time  // omega in rad/s
            );
            
            // Apply light smoothing for more realistic velocity changes
            double smooth_factor = 0.1; // Light smoothing
            if (smooth_factor > 0.0) {
                chassis_speeds = new ChassisSpeeds(
                    chassis_speeds.vxMetersPerSecond * smooth_factor + target_speeds.vxMetersPerSecond * (1.0 - smooth_factor),
                    chassis_speeds.vyMetersPerSecond * smooth_factor + target_speeds.vyMetersPerSecond * (1.0 - smooth_factor),
                    chassis_speeds.omegaRadiansPerSecond * smooth_factor + target_speeds.omegaRadiansPerSecond * (1.0 - smooth_factor)
                );
            } else {
                chassis_speeds = target_speeds;
            }
        }
    }

    /**
     * Returns the swerve drive kinematics instance
     *
     * @return SwerveDriveKinematics object representing the swerve drive
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics_;
    }

    /**
     * Sets the drive position gains for all modules.
     *
     * @param gains The SlotConfigs containing the position gains.
     */
    private void setDrivePositionGains(SlotConfigs gains) {
        setDriveGains(0, gains);
    }

    /**
     * Sets the drive velocity gains for all modules.
     *
     * @param gains The SlotConfigs containing the velocity gains.
     */
    private void setDriveVelocityGains(SlotConfigs gains) {
        setDriveGains(1, gains);
    }

    /**
     * Sets the drive gains for all modules.
     *
     * @param slot The slot number to set the gains for.
     * @param gains The SlotConfigs containing the gains.
     */
    private void setDriveGains(int slot, SlotConfigs gains) {
        for (var module : modules_) {
            module.setDriveGains(slot, gains);
        }
    }

    /**
     * Sets the steer gains for all modules.
     *
     * @param gains The SlotConfigs containing the steer gains.
     */
    private void setSteerGains(SlotConfigs gains) {
        for (var module : modules_) {
            module.setSteerGains(gains);
        }
    }
}
