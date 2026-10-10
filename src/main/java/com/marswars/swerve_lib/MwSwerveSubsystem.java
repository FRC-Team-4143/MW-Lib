package com.marswars.swerve_lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.marswars.auto.ChoreoEventTracker;
import com.marswars.auto.ChoreoTrajectory;
import com.marswars.logging.MwLog;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.marswars.swerve_lib.ChassisRequest.XPositiveReference;
import com.marswars.swerve_lib.module.Module.DriveControlMode;
import com.marswars.swerve_lib.module.Module.SteerControlMode;
import com.marswars.util.DynamicSlewRateLimiter;
import com.marswars.util.TunablePid;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Generic swerve drive subsystem: teleop (field/robot centric), crawl, rotation lock, Choreo path
 * following, tractor beam, brake and tuning states on top of {@link SwerveMech}.
 *
 * <p>Robots extend this with their own {@code SwerveSubsystem} (keeping the class name so the
 * logging keys stay {@code Subsystem/Swerve/...}), supply the robot pose and driver inputs, and
 * own the singleton. Driver-preference policies (e.g. "POV forces crawl") go in {@link
 * #remapWantedState(SwerveStates)}.
 *
 * @param <C> the robot's swerve constants type
 */
public abstract class MwSwerveSubsystem<C extends MwSwerveConstants>
        extends MwSubsystem<SwerveStates, C> {

    // External inputs
    private final Supplier<Pose2d> pose_supplier_;
    private final SwerveDriverInputs driver_inputs_;

    // State Specific Members
    Trajectory<SwerveSample> desired_choreo_traj_;
    private final Timer choreo_timer_ = new Timer();
    private Optional<SwerveSample> choreo_sample_to_apply_;
    private final ChoreoEventTracker choreo_event_tracker_;
    private final PIDController choreo_x_controller_ =
            new PIDController(
                    CONSTANTS.CHOREO_TRANSLATION_CONTROLLER_KP,
                    CONSTANTS.CHOREO_TRANSLATION_CONTROLLER_KI,
                    CONSTANTS.CHOREO_TRANSLATION_CONTROLLER_KD);
    private final PIDController choreo_y_controller_ =
            new PIDController(
                    CONSTANTS.CHOREO_TRANSLATION_CONTROLLER_KP,
                    CONSTANTS.CHOREO_TRANSLATION_CONTROLLER_KI,
                    CONSTANTS.CHOREO_TRANSLATION_CONTROLLER_KD);
    private final PIDController choreo_theta_controller_ =
            new PIDController(
                    CONSTANTS.CHOREO_THETA_CONTROLLER_KP,
                    CONSTANTS.CHOREO_THETA_CONTROLLER_KI,
                    CONSTANTS.CHOREO_THETA_CONTROLLER_KD);

    private Pose2d desired_tractor_beam_pose_ = new Pose2d();
    private double max_lin_vel_for_tractor_beam_;
    private double max_ang_vel_for_tractor_beam_;
    private final PIDController tractor_beam_controller_ =
            new PIDController(
                    CONSTANTS.TRACTOR_BEAM_CONTROLLER_KP,
                    CONSTANTS.TRACTOR_BEAM_CONTROLLER_KI,
                    CONSTANTS.TRACTOR_BEAM_CONTROLLER_KD);
    private Rotation2d desired_rotation_lock_rot_ = new Rotation2d();
    private Translation2d desired_rotation_lock_cor_ = new Translation2d();
    private double desired_rotation_lock_feedforward_ = 0.0;
    private ChassisSpeeds desired_chassis_speeds_ = new ChassisSpeeds(0, 0, 0);

    // Telop Smoothness controllers and scalars
    private double tele_op_velocity_scalar_ = 1.0;
    private double tele_op_velocity_rl_scalar_ = 1.0;
    private DynamicSlewRateLimiter x_tele_op_velocity_slew_limiter_ =
            new DynamicSlewRateLimiter(CONSTANTS.MAX_TRANSLATION_ACCEL * 1.0);
    private DynamicSlewRateLimiter y_tele_op_velocity_slew_limiter_ =
            new DynamicSlewRateLimiter(CONSTANTS.MAX_TRANSLATION_ACCEL * 1.0);

    // IO Members
    private SwerveMech swerve_mech_;
    private Rotation2d operator_forward_direction_ = OperatorPerspective.BLUE_ALLIANCE.heading;

    private ChassisRequest.FieldCentric field_centric_request_;
    private ChassisRequest.RobotCentric robot_centric_request_;
    private ChassisRequest.FieldCentricFacingAngle choreo_rotation_lock_request_;
    private ChassisRequest.FieldCentricFacingAngle field_centric_rotation_lock_request_;
    private ChassisRequest.RobotCentricFacingAngle robot_centric_rotation_lock_request_;
    private ChassisRequest.ApplyFieldSpeeds field_speeds_request_;
    private ChassisRequest.ApplyChassisSpeeds chassis_speeds_request_;
    private ChassisRequest.SwerveDriveBrake brake_request_;

    // Shared by every rotation-lock request so tuning and isAtDesiredRotation see one controller
    private final PhoenixPIDController heading_controller_;

    /**
     * Creates the swerve subsystem and its {@link SwerveMech}.
     *
     * @param constants the robot's swerve constants
     * @param pose_supplier the robot's field pose (typically the localization estimate). Called
     *     lazily, so it may reference subsystems constructed after this one.
     * @param driver_inputs the driver joystick inputs used by the teleop and crawl states
     */
    protected MwSwerveSubsystem(
            C constants, Supplier<Pose2d> pose_supplier, SwerveDriverInputs driver_inputs) {
        super(SwerveStates.IDLE, constants);
        pose_supplier_ = pose_supplier;
        driver_inputs_ = driver_inputs;

        swerve_mech_ = new SwerveMech(getSubsystemKey(), CONSTANTS.getDriveConfig());

        heading_controller_ =
                new PhoenixPIDController(
                        CONSTANTS.HEADING_CONTROLLER_KP,
                        CONSTANTS.HEADING_CONTROLLER_KI,
                        CONSTANTS.HEADING_CONTROLLER_KD);

        // Initialize event tracker with pose supplier
        choreo_event_tracker_ =
                new ChoreoEventTracker(getSubsystemKey() + "Choreo/Events/", pose_supplier_);
        choreo_theta_controller_.enableContinuousInput(-Math.PI, Math.PI);
        // Initialize drive mode requests
        field_centric_request_ =
                new ChassisRequest.FieldCentric()
                        .withDriveRequestType(DriveControlMode.OPEN_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP)
                        .withDeadband(CONSTANTS.MAX_TRANSLATION_RATE * 0.01)
                        .withRotationalDeadband(CONSTANTS.MAX_ANGULAR_RATE * 0.01)
                        .withXPositiveReference(XPositiveReference.OperatorPerspective);
        robot_centric_request_ =
                new ChassisRequest.RobotCentric()
                        .withDriveRequestType(DriveControlMode.OPEN_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP)
                        .withDeadband(CONSTANTS.MAX_TRANSLATION_RATE * 0.01)
                        .withRotationalDeadband(CONSTANTS.MAX_ANGULAR_RATE * 0.01);
        field_centric_rotation_lock_request_ =
                new ChassisRequest.FieldCentricFacingAngle()
                        .withDriveRequestType(DriveControlMode.OPEN_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP)
                        .withDeadband(CONSTANTS.MAX_TRANSLATION_RATE * 0.01)
                        .withRotationalDeadband(CONSTANTS.MAX_ANGULAR_RATE * 0.01)
                        .withHeadingController(heading_controller_)
                        .withXPositiveReference(XPositiveReference.OperatorPerspective);
        choreo_rotation_lock_request_ =
                new ChassisRequest.FieldCentricFacingAngle()
                        .withDriveRequestType(DriveControlMode.CLOSED_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP)
                        .withDeadband(CONSTANTS.MAX_TRANSLATION_RATE * 0.01)
                        .withRotationalDeadband(CONSTANTS.MAX_ANGULAR_RATE * 0.01)
                        .withHeadingController(heading_controller_)
                        .withXPositiveReference(XPositiveReference.TowardsRedAlliance);
        robot_centric_rotation_lock_request_ =
                new ChassisRequest.RobotCentricFacingAngle()
                        .withDriveRequestType(DriveControlMode.CLOSED_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP)
                        .withDeadband(CONSTANTS.MAX_TRANSLATION_RATE * 0.01)
                        .withRotationalDeadband(CONSTANTS.MAX_ANGULAR_RATE * 0.01)
                        .withHeadingController(heading_controller_);
        field_speeds_request_ =
                new ChassisRequest.ApplyFieldSpeeds()
                        .withDriveRequestType(DriveControlMode.CLOSED_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP);
        chassis_speeds_request_ =
                new ChassisRequest.ApplyChassisSpeeds()
                        .withDriveRequestType(DriveControlMode.CLOSED_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP);
        brake_request_ = new ChassisRequest.SwerveDriveBrake();

        TunablePid.create(getSubsystemKey() + "TractorBeam/Gains", tractor_beam_controller_);
        TunablePid.create(
                getSubsystemKey() + "ChoreoPath/Translation/Gains",
                choreo_x_controller_,
                choreo_y_controller_);
        TunablePid.create(
                getSubsystemKey() + "ChoreoPath/Rotation/Gains", choreo_theta_controller_);
        TunablePid.create(
                getSubsystemKey() + "RotationLock/Gains",
                field_centric_rotation_lock_request_.HeadingController);
        MwLog.tunable(
                getSubsystemKey() + "VelocityScalar",
                tele_op_velocity_rl_scalar_,
                this::setTeleOpVelocityScalar);
        MwLog.tunable(
                getSubsystemKey() + "VelocityRLScalar",
                tele_op_velocity_rl_scalar_,
                this::setTeleOpVelocityRateLimitScalar);
    }

    // reset
    @Override
    public void reset() {
        system_state_ = SwerveStates.IDLE;
    }

    // getIos
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(swerve_mech_);
    }

    /**
     * Hook to rewrite the wanted state before the transition is handled, every loop. Use it for
     * driver-preference policies such as forcing a crawl state while the POV is pressed (see
     * {@link #crawlVariantOf(SwerveStates)}). The default returns {@code wanted} unchanged.
     *
     * @param wanted the state requested through {@link #setWantedState}
     * @return the state to actually transition to
     */
    protected SwerveStates remapWantedState(SwerveStates wanted) {
        return wanted;
    }

    /**
     * Maps a state to the crawl state that preserves its frame and rotation lock: field-centric
     * states map to field-centric crawl, rotation-locked states to rotation-locked crawl, and
     * everything else to robot-centric crawl.
     *
     * @param state the state to map
     * @return the matching crawl state
     */
    protected static SwerveStates crawlVariantOf(SwerveStates state) {
        return switch (state) {
            case FIELD_CENTRIC_ROTATION_LOCK, CRAWL_FIELD_CENTRIC_ROTATION_LOCK ->
                    SwerveStates.CRAWL_FIELD_CENTRIC_ROTATION_LOCK;
            case ROBOT_CENTRIC_ROTATION_LOCK,
                    CHASSIS_SPEEDS_ROTATION_LOCK,
                    CRAWL_ROBOT_CENTRIC_ROTATION_LOCK ->
                    SwerveStates.CRAWL_ROBOT_CENTRIC_ROTATION_LOCK;
            case FIELD_CENTRIC, CRAWL_FIELD_CENTRIC -> SwerveStates.CRAWL_FIELD_CENTRIC;
            default -> SwerveStates.CRAWL_ROBOT_CENTRIC;
        };
    }

    @Override
    protected void handleStateTransition(SwerveStates wanted_state) {
        wanted_state = remapWantedState(wanted_state);

        // Stop event tracker if leaving choreo states
        if ((system_state_ == SwerveStates.CHOREO_PATH
                        || system_state_ == SwerveStates.CHOREO_PATH_ROTATION_LOCK)
                && wanted_state != SwerveStates.CHOREO_PATH
                && wanted_state != SwerveStates.CHOREO_PATH_ROTATION_LOCK) {
            choreo_timer_.stop();
            choreo_event_tracker_.stop();
        }

        system_state_ =
                switch (wanted_state) {
                    case ROBOT_CENTRIC -> SwerveStates.ROBOT_CENTRIC;
                    case FIELD_CENTRIC -> SwerveStates.FIELD_CENTRIC;
                    case CHOREO_PATH -> {
                        // If we are not already in a choreo path state, restart the timer
                        // The additional check is needed to prevent resetting the timer when
                        // switching between the two choreo states
                        if (system_state_ != SwerveStates.CHOREO_PATH
                                && system_state_ != SwerveStates.CHOREO_PATH_ROTATION_LOCK) {
                            choreo_x_controller_.reset();
                            choreo_y_controller_.reset();
                            choreo_theta_controller_.reset();
                            choreo_timer_.restart();
                            choreo_event_tracker_.start();
                            yield SwerveStates.CHOREO_PATH;
                        } else {
                            yield SwerveStates.CHOREO_PATH;
                        }
                    }
                    case TRACTOR_BEAM -> SwerveStates.TRACTOR_BEAM;
                    case CHASSIS_SPEEDS -> SwerveStates.CHASSIS_SPEEDS;
                    case CRAWL_ROBOT_CENTRIC -> SwerveStates.CRAWL_ROBOT_CENTRIC;
                    case CRAWL_FIELD_CENTRIC -> SwerveStates.CRAWL_FIELD_CENTRIC;
                    case ROBOT_CENTRIC_ROTATION_LOCK -> SwerveStates.ROBOT_CENTRIC_ROTATION_LOCK;
                    case FIELD_CENTRIC_ROTATION_LOCK -> SwerveStates.FIELD_CENTRIC_ROTATION_LOCK;
                    case CHOREO_PATH_ROTATION_LOCK -> {
                        // If we are not already in a choreo path state, restart the timer
                        // The additional check is needed to prevent resetting the timer when
                        // switching between the two choreo states
                        if (system_state_ != SwerveStates.CHOREO_PATH_ROTATION_LOCK
                                && system_state_ != SwerveStates.CHOREO_PATH) {
                            choreo_x_controller_.reset();
                            choreo_y_controller_.reset();
                            choreo_theta_controller_.reset();
                            choreo_timer_.restart();
                            choreo_event_tracker_.start();
                            choreo_sample_to_apply_ =
                                    desired_choreo_traj_.sampleAt(choreo_timer_.get(), false);
                            yield SwerveStates.CHOREO_PATH_ROTATION_LOCK;
                        } else {
                            choreo_sample_to_apply_ =
                                    desired_choreo_traj_.sampleAt(choreo_timer_.get(), false);
                            yield SwerveStates.CHOREO_PATH_ROTATION_LOCK;
                        }
                    }
                    case CRAWL_ROBOT_CENTRIC_ROTATION_LOCK ->
                            SwerveStates.CRAWL_ROBOT_CENTRIC_ROTATION_LOCK;
                    case CRAWL_FIELD_CENTRIC_ROTATION_LOCK ->
                            SwerveStates.CRAWL_FIELD_CENTRIC_ROTATION_LOCK;
                    case CHASSIS_SPEEDS_ROTATION_LOCK -> SwerveStates.CHASSIS_SPEEDS_ROTATION_LOCK;
                    case TUNING -> SwerveStates.TUNING;
                    case BRAKE -> SwerveStates.BRAKE;
                    default -> SwerveStates.IDLE;
                };
    }

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {
        ChassisSpeeds controller_inputs = calculateSpeedsBasedOnJoystickInputs();
        // Update the request to apply based on the system state
        switch (system_state_) {
            case ROBOT_CENTRIC:
                swerve_mech_.setChassisRequest(
                        robot_centric_request_.withSpeeds(controller_inputs));
                desired_chassis_speeds_ = controller_inputs;
                break;
            case FIELD_CENTRIC:
                swerve_mech_.setChassisRequest(
                        field_centric_request_.withSpeeds(controller_inputs));
                desired_chassis_speeds_ = removeOperatorPerspective(controller_inputs);
                break;
            case CHOREO_PATH:
                choreoPathState();
                break;
            case TRACTOR_BEAM:
                tractorBeamState();
                break;
            case CHASSIS_SPEEDS:
                swerve_mech_.setChassisRequest(
                        chassis_speeds_request_.withSpeeds(desired_chassis_speeds_));
                break;
            case CRAWL_ROBOT_CENTRIC:
                handleCrawlState(false);
                break;
            case CRAWL_FIELD_CENTRIC:
                handleFieldCentricCrawlState(false);
                break;
            case ROBOT_CENTRIC_ROTATION_LOCK:
                swerve_mech_.setChassisRequest(
                        robot_centric_rotation_lock_request_
                                .withTargetHeading(desired_rotation_lock_rot_)
                                .withSpeeds(controller_inputs)
                                .withCenterOfRotation(desired_rotation_lock_cor_)
                                .withHeadingFeedforward(desired_rotation_lock_feedforward_));
                desired_chassis_speeds_ = controller_inputs;
                MwLog.log(
                        getSubsystemKey() + "RotationLock/Rotation",
                        desired_rotation_lock_rot_.getRadians(),
                        Radians);
                MwLog.log(getSubsystemKey() + "RotationLock/COR", desired_rotation_lock_cor_);
                break;
            case FIELD_CENTRIC_ROTATION_LOCK:
                swerve_mech_.setChassisRequest(
                        field_centric_rotation_lock_request_
                                .withTargetHeading(desired_rotation_lock_rot_)
                                .withSpeeds(controller_inputs)
                                .withCenterOfRotation(desired_rotation_lock_cor_)
                                .withHeadingFeedforward(desired_rotation_lock_feedforward_));
                desired_chassis_speeds_ = controller_inputs;
                MwLog.log(
                        getSubsystemKey() + "RotationLock/Rotation",
                        desired_rotation_lock_rot_.getRadians(),
                        Radians);
                MwLog.log(getSubsystemKey() + "RotationLock/COR", desired_rotation_lock_cor_);
                desired_chassis_speeds_ = removeOperatorPerspective(controller_inputs);
                break;
            case CHOREO_PATH_ROTATION_LOCK:
                choreoPathRotationLockState();
                break;
            case CRAWL_ROBOT_CENTRIC_ROTATION_LOCK:
                handleCrawlState(true);
                break;
            case CRAWL_FIELD_CENTRIC_ROTATION_LOCK:
                handleFieldCentricCrawlState(true);
                break;
            case CHASSIS_SPEEDS_ROTATION_LOCK:
                swerve_mech_.setChassisRequest(
                        robot_centric_rotation_lock_request_
                                .withTargetHeading(desired_rotation_lock_rot_)
                                .withSpeeds(desired_chassis_speeds_)
                                .withCenterOfRotation(desired_rotation_lock_cor_)
                                .withHeadingFeedforward(desired_rotation_lock_feedforward_));
                MwLog.log(getSubsystemKey() + "RotationLock/ChassisSpeed", desired_chassis_speeds_);
                MwLog.log(
                        getSubsystemKey() + "RotationLock/Rotation",
                        desired_rotation_lock_rot_.getRadians(),
                        Radians);
                MwLog.log(getSubsystemKey() + "RotationLock/COR", desired_rotation_lock_cor_);
                break;
            case TUNING:
                swerve_mech_.setChassisRequest(
                        chassis_speeds_request_.withSpeeds(desired_chassis_speeds_));
                break;
            case BRAKE:
                swerve_mech_.setChassisRequest(brake_request_);
                desired_chassis_speeds_ = removeOperatorPerspective(controller_inputs);
                break;
            case IDLE:
            default:
                desired_chassis_speeds_ = new ChassisSpeeds();
                swerve_mech_.setChassisRequest(
                        new ChassisRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds()));
                break;
        }
        // Set state static request parameters
        swerve_mech_.setChassisRequestParameters(
                getFieldPose(), operator_forward_direction_);
        MwLog.log(getSubsystemKey() + "DesiredChassisSpeeds", desired_chassis_speeds_);
    }

    // =============================================================================
    // PUBLIC HELPER METHODS

    // =============================================================================

    /**
     * Handles the TRACTOR_BEAM state by calculating the necessary chassis speeds to move towards
     * the desired tractor beam pose.
     */
    private void tractorBeamState() {
        Translation2d translation_to_desired_point =
                desired_tractor_beam_pose_
                        .getTranslation()
                        .minus(getFieldPose().getTranslation());
        double linear_distance = translation_to_desired_point.getNorm();
        double friction_constant = 0.0;
        if (linear_distance >= Units.inchesToMeters(0.5)) {
            friction_constant =
                    CONSTANTS.TRACTOR_BEAM_STATIC_FRICTION_CONSTANT
                            * CONSTANTS.MAX_TRANSLATION_RATE;
        }
        Rotation2d direction_of_travel = translation_to_desired_point.getAngle();
        double velocity_output =
                Math.min(
                        Math.abs(tractor_beam_controller_.calculate(linear_distance, 0))
                                + friction_constant,
                        max_lin_vel_for_tractor_beam_);
        double x_component = velocity_output * direction_of_travel.getCos();
        double y_component = velocity_output * direction_of_travel.getSin();

        MwLog.log(
                getSubsystemKey() + "TractorBeam/XVelocityComponent", x_component, MetersPerSecond);
        MwLog.log(
                getSubsystemKey() + "TractorBeam/YVelocityComponent", y_component, MetersPerSecond);
        MwLog.log(
                getSubsystemKey() + "TractorBeam/VelocityOutput", velocity_output, MetersPerSecond);
        MwLog.log(getSubsystemKey() + "TractorBeam/LinearDistance", linear_distance, Meters);
        MwLog.log(
                getSubsystemKey() + "TractorBeam/DirectionOfTravel",
                direction_of_travel.getRadians(),
                Radians);
        MwLog.log(getSubsystemKey() + "TractorBeam/DesiredPose", desired_tractor_beam_pose_);

        desired_chassis_speeds_ = new ChassisSpeeds(x_component, y_component, 0.0);
        if (Double.isNaN(max_ang_vel_for_tractor_beam_)) {
            swerve_mech_.setChassisRequest(
                    field_centric_rotation_lock_request_
                            .withSpeeds(desired_chassis_speeds_)
                            .withTargetHeading(desired_tractor_beam_pose_.getRotation()));
        } else {
            swerve_mech_.setChassisRequest(
                    field_centric_rotation_lock_request_
                            .withSpeeds(desired_chassis_speeds_)
                            .withTargetHeading(desired_tractor_beam_pose_.getRotation())
                            .withMaxAbsRotationalRate(max_ang_vel_for_tractor_beam_));
        }
    }

    private Pose2d choreoPathCommon() {
        Pose2d pose = getFieldPose();

        choreo_sample_to_apply_ = desired_choreo_traj_.sampleAt(choreo_timer_.get(), false);

        choreo_event_tracker_.update(choreo_timer_.get());
        if (choreo_sample_to_apply_.isPresent()) {
            SwerveSample sample = choreo_sample_to_apply_.get();
            MwLog.log(getSubsystemKey() + "Choreo/TimerValue", choreo_timer_.get(), Seconds);
            MwLog.log(getSubsystemKey() + "Choreo/TrajName", desired_choreo_traj_.name());
            MwLog.log(
                    getSubsystemKey() + "Choreo/TotalTime",
                    desired_choreo_traj_.getTotalTime(),
                    Seconds);
            // if distance threshold is exceded stop the timer so you do not get so far ahead
            if (sample.getPose().getTranslation().getDistance(pose.getTranslation())
                    > CONSTANTS.CHOREO_LOOK_AHEAD) {
                choreo_timer_.stop();
            } else {
                choreo_timer_.start();
            }
        }
        return pose;
    }

    /**
     * Handles the CHOREO_PATH state by applying the appropriate chassis speeds based on the current
     * trajectory sample and PID controller outputs.
     */
    private void choreoPathState() {
        Pose2d pose = choreoPathCommon();

        if (choreo_sample_to_apply_.isPresent()) {
            SwerveSample sample = choreo_sample_to_apply_.get();
            MwLog.log(getSubsystemKey() + "Choreo/sample/DesiredPose", sample.getPose());
            MwLog.log(
                    getSubsystemKey() + "Choreo/sample/DesiredChassisSpeeds",
                    sample.getChassisSpeeds());

            ChassisSpeeds target_speeds = sample.getChassisSpeeds();
            target_speeds.vxMetersPerSecond +=
                    choreo_x_controller_.calculate(pose.getX(), sample.x);
            target_speeds.vyMetersPerSecond +=
                    choreo_y_controller_.calculate(pose.getY(), sample.y);
            target_speeds.omegaRadiansPerSecond +=
                    choreo_theta_controller_.calculate(
                            pose.getRotation().getRadians(), sample.heading);

            desired_chassis_speeds_ = target_speeds;
            swerve_mech_.setChassisRequest(
                    field_speeds_request_.withSpeeds(desired_chassis_speeds_));
        } else {
            // If no sample is available, we will just stop the robot
            desired_chassis_speeds_ = new ChassisSpeeds();
            swerve_mech_.setChassisRequest(new ChassisRequest.Idle());
        }
    }

    /**
     * Handles the CHOREO_PATH_ROTATION_LOCK state by applying chassis speeds based on the current
     * trajectory sample, but overriding the rotation to a fixed desired rotation.
     */
    private void choreoPathRotationLockState() {
        Pose2d pose = choreoPathCommon();

        if (choreo_sample_to_apply_.isPresent()) {
            SwerveSample sample = choreo_sample_to_apply_.get();
            // Generate new pose with overridden rotation
            Pose2d overridden_pose =
                    new Pose2d(sample.getPose().getTranslation(), desired_rotation_lock_rot_);
            MwLog.log(getSubsystemKey() + "Choreo/sample/DesiredPose", overridden_pose);
            // Generate new chassis speeds with overridden rotation speed
            ChassisSpeeds overridden_speeds = sample.getChassisSpeeds();
            overridden_speeds.omegaRadiansPerSecond = 0.0;
            MwLog.log(getSubsystemKey() + "Choreo/sample/DesiredChassisSpeeds", overridden_speeds);

            ChassisSpeeds target_speeds = overridden_speeds;
            target_speeds.vxMetersPerSecond +=
                    choreo_x_controller_.calculate(pose.getX(), sample.x);
            target_speeds.vyMetersPerSecond +=
                    choreo_y_controller_.calculate(pose.getY(), sample.y);

            desired_chassis_speeds_ = target_speeds;
            swerve_mech_.setChassisRequest(
                    choreo_rotation_lock_request_
                            .withSpeeds(desired_chassis_speeds_)
                            .withTargetHeading(desired_rotation_lock_rot_));
        } else {
            // If no sample is available, we will just stop the robot
            desired_chassis_speeds_ = new ChassisSpeeds();
            swerve_mech_.setChassisRequest(new ChassisRequest.Idle());
        }
    }

    /**
     * Handles crawl states (robot-centric POV movement at reduced speed).
     *
     * @param is_rotation_locked whether the rotation should be locked to a target heading
     */
    private void handleCrawlState(boolean is_rotation_locked) {
        desired_chassis_speeds_ = calculateSpeedBasedOnPOVInputs();
        if (is_rotation_locked) {
            swerve_mech_.setChassisRequest(
                    robot_centric_rotation_lock_request_
                            .withTargetHeading(desired_rotation_lock_rot_)
                            .withSpeeds(desired_chassis_speeds_)
                            .withCenterOfRotation(desired_rotation_lock_cor_)
                            .withHeadingFeedforward(desired_rotation_lock_feedforward_));
            MwLog.log(
                    getSubsystemKey() + "RotationLock/Rotation",
                    desired_rotation_lock_rot_.getRadians(),
                    Radians);
            MwLog.log(getSubsystemKey() + "RotationLock/COR", desired_rotation_lock_cor_);
        } else {
            swerve_mech_.setChassisRequest(
                    robot_centric_request_.withSpeeds(desired_chassis_speeds_));
        }
    }

    /**
     * Handles field-centric crawl states (POV movement at reduced speed relative to field).
     *
     * @param is_rotation_locked whether the rotation should be locked to a target heading
     */
    private void handleFieldCentricCrawlState(boolean is_rotation_locked) {
        desired_chassis_speeds_ = calculateSpeedBasedOnPOVInputs();
        if (is_rotation_locked) {
            swerve_mech_.setChassisRequest(
                    field_centric_rotation_lock_request_
                            .withTargetHeading(desired_rotation_lock_rot_)
                            .withSpeeds(desired_chassis_speeds_)
                            .withCenterOfRotation(desired_rotation_lock_cor_)
                            .withHeadingFeedforward(desired_rotation_lock_feedforward_));
            MwLog.log(
                    getSubsystemKey() + "RotationLock/Rotation",
                    desired_rotation_lock_rot_.getRadians(),
                    Radians);
            MwLog.log(getSubsystemKey() + "RotationLock/COR", desired_rotation_lock_cor_);
        } else {
            swerve_mech_.setChassisRequest(
                    field_centric_request_.withSpeeds(desired_chassis_speeds_));
        }
    }

    // ------------------------------------------------
    // Chassis Control Methods
    // ------------------------------------------------

    /**
     * Updates the internal target for the robot to follow in CHOREO_PATH or
     * CHOREO_PATH_ROTATION_LOCK
     *
     * @param trajectory the trajectory for the robot to follow
     */
    public void setDesiredChoreoTrajectory(ChoreoTrajectory trajectory) {
        desired_choreo_traj_ = trajectory.getTrajectory();

        // Load events from the trajectory (passes trajectory so poses can be extracted)
        choreo_event_tracker_.setEvents(trajectory);

        // Reset the timer if we are already in a choreo path state to restart the new
        // trajectory
        if (system_state_ == SwerveStates.CHOREO_PATH
                || system_state_ == SwerveStates.CHOREO_PATH_ROTATION_LOCK) {
            choreo_timer_.reset();
            choreo_event_tracker_.start();
            choreo_x_controller_.reset();
            choreo_y_controller_.reset();
            choreo_theta_controller_.reset();
        }

        // Log the trajectory poses for debugging
        MwLog.log(getSubsystemKey() + "Choreo/Trajectory", desired_choreo_traj_.getPoses());
    }

    /**
     * Command version of {@link #setDesiredChoreoTrajectory(ChoreoTrajectory)}
     *
     * @param trajectory the trajectory for the robot to follow
     * @return A command that sets the desired choreo trajectory
     */
    public Command setDesiredChoreoTrajectoryCommand(Supplier<ChoreoTrajectory> trajectory) {
        return Commands.runOnce(() -> setDesiredChoreoTrajectory(trajectory.get()));
    }

    /**
     * Updates the internal target for the robot to reach in TRACTOR_BEAM
     *
     * @param target_pose target pose for the robot to reach
     */
    public void setDesiredTractorBeamPose(Pose2d pose) {
        desired_tractor_beam_pose_ = pose;
        max_lin_vel_for_tractor_beam_ = CONSTANTS.MAX_TRANSLATION_RATE;
        max_ang_vel_for_tractor_beam_ = Double.NaN;
    }

    /**
     * Updates the internal target for the robot to reach in TRACTOR_BEAM
     *
     * @param pose target pose for the robot to reach
     * @param max_lin_vel maximum linear velocity for the robot to reach the target pose
     */
    public void setDesiredTractorBeamPoseWithMaxLinVel(Pose2d pose, double max_lin_vel) {
        max_lin_vel_for_tractor_beam_ = max_lin_vel;
        max_ang_vel_for_tractor_beam_ = Double.NaN;
        desired_tractor_beam_pose_ = pose;
    }

    /**
     * Updates the internal target for the robot to reach in TRACTOR_BEAM
     *
     * @param pose target pose for the robot to reach
     * @param max_ang_vel maximum angular velocity for the robot to reach the target pose
     */
    public void setDesiredTractorBeamPoseWithMaxAngVel(Pose2d pose, double max_ang_vel) {
        max_lin_vel_for_tractor_beam_ = CONSTANTS.MAX_TRANSLATION_RATE;
        max_ang_vel_for_tractor_beam_ = max_ang_vel;
        desired_tractor_beam_pose_ = pose;
    }

    /**
     * Updates the internal target for the robot to reach in TRACTOR_BEAM
     *
     * @param pose target pose for the robot to reach
     * @param max_lin_vel maximum linear velocity for the robot to reach the target pose
     * @param max_ang_vel maximum angular velocity for the robot to reach the target pose
     */
    public void setTractorBeamPoseWithConstraints(
            Pose2d pose, double max_lin_vel, double max_ang_vel) {
        max_lin_vel_for_tractor_beam_ = max_lin_vel;
        max_ang_vel_for_tractor_beam_ = max_ang_vel;
        desired_tractor_beam_pose_ = pose;
    }

    /**
     * Updates the internal target for the robot to follow in CHASSIS_SPEED_ROTATION_LOCK
     *
     * @param speeds desired chassis speeds
     * @param rotation desired rotation to lock to
     */
    public void setDesiredChassisSpeed(ChassisSpeeds speeds) {
        desired_chassis_speeds_ = speeds;
    }

    /**
     * Updates the internal target for the robot to face turing around a desired center point in
     * FIELD_CENTRIC_ROTATION_LOCK or CHOREO_PATH_ROTATION_LOCK
     *
     * @param rotation desired rotation to lock to
     */
    public void setDesiredRotationLock(Rotation2d rotation) {
        setDesiredRotationLockCOR(rotation, Translation2d.kZero);
    }

    /**
     * Updates the internal target for the robot to face turing around a desired center point in
     * FIELD_CENTRIC_ROTATION_LOCK or CHOREO_PATH_ROTATION_LOCK
     *
     * @param rotation desired rotation to lock to
     * @param center_point desired center point to rotate around
     */
    public void setDesiredRotationLockCOR(Rotation2d rotation, Translation2d center_point) {
        desired_rotation_lock_rot_ = rotation;
        desired_rotation_lock_cor_ = center_point;
        desired_rotation_lock_feedforward_ = 0.0;
    }

    /**
     * Updates the internal target for the robot to face with feedforward, turning around a desired
     * center point in FIELD_CENTRIC_ROTATION_LOCK or CHOREO_PATH_ROTATION_LOCK
     *
     * @param rotation desired rotation to lock to
     * @param center_point desired center point to rotate around
     * @param feedforward feedforward rotational velocity in rad/s
     */
    public void setDesiredRotationLockCORWithFF(
            Rotation2d rotation, Translation2d center_point, double feedforward) {
        desired_rotation_lock_rot_ = rotation;
        desired_rotation_lock_cor_ = center_point;
        desired_rotation_lock_feedforward_ = feedforward;
    }

    /**
     * Command version of {@link #setDesiredChassisSpeed(ChassisSpeeds)} that sets the desired
     * chassis speeds for tuning purposes.
     *
     * @param speeds desired chassis speeds (robot relative)
     * @return a command that sets the desired chassis speeds
     */
    public Command chassisTuningCommand(ChassisSpeeds speeds) {
        return Commands.startEnd(
                        () -> {
                            setDesiredChassisSpeed(speeds);
                            setWantedState(SwerveStates.TUNING);
                        },
                        () -> {
                            setWantedState(SwerveStates.FIELD_CENTRIC);
                        })
                .withName(
                        "Chassis Tuning : X="
                                + speeds.vxMetersPerSecond
                                + " Y="
                                + speeds.vyMetersPerSecond
                                + " Omega="
                                + speeds.omegaRadiansPerSecond);
    }

    /**
     * Toggles between FIELD_CENTRIC and ROBOT_CENTRIC modes.
     *
     * @return A command that toggles the field centric mode
     */
    public Command toggleFieldCentric() {
        String new_mode =
                (system_state_ == SwerveStates.FIELD_CENTRIC) ? "ROBOT_CENTRIC" : "FIELD_CENTRIC";
        return Commands.runOnce(
                        () -> {
                            if (system_state_ == SwerveStates.FIELD_CENTRIC) {
                                setWantedState(SwerveStates.ROBOT_CENTRIC);
                            } else {
                                setWantedState(SwerveStates.FIELD_CENTRIC);
                            }
                        })
                .withName("Toggle Field Centric: " + new_mode);
    }

    // ------------------------------------------------
    // Operator Interface Methods
    // ------------------------------------------------

    /**
     * Calculates chassis speeds based on joystick inputs.
     *
     * @return the controller inputs as a ChassisSpeeds object, where the x and y components
     *     represent the translation speeds and the omega component represents the angular speed
     */
    private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
        if (DriverStation.getAlliance().isEmpty()) {
            return new ChassisSpeeds();
        }

        double x_magnitude =
                -MathUtil.applyDeadband(driver_inputs_.left_y().getAsDouble(), CONSTANTS.CONTROLLER_DEADBAND);
        double y_magnitude =
                -MathUtil.applyDeadband(driver_inputs_.left_x().getAsDouble(), CONSTANTS.CONTROLLER_DEADBAND);
        double angular_magnitude =
                -MathUtil.applyDeadband(
                        driver_inputs_.right_x().getAsDouble(), CONSTANTS.CONTROLLER_DEADBAND);
        MwLog.log(
                getSubsystemKey() + "JoystickRaw",
                new ChassisSpeeds(x_magnitude, y_magnitude, angular_magnitude));

        // Calculate base inputs as velocities (apply scalar)
        x_magnitude = x_magnitude * CONSTANTS.MAX_TRANSLATION_RATE * tele_op_velocity_scalar_;
        y_magnitude = y_magnitude * CONSTANTS.MAX_TRANSLATION_RATE * tele_op_velocity_scalar_;
        angular_magnitude = angular_magnitude * CONSTANTS.MAX_ANGULAR_RATE;

        // Apply slew rate limiters to smooth inputs
        x_magnitude = x_tele_op_velocity_slew_limiter_.calculate(x_magnitude);
        y_magnitude = y_tele_op_velocity_slew_limiter_.calculate(y_magnitude);

        ChassisSpeeds speeds = new ChassisSpeeds(x_magnitude, y_magnitude, angular_magnitude);
        MwLog.log(getSubsystemKey() + "JoystickFiltered", speeds);
        return speeds;
    }

    /**
     * Calculates chassis speeds based on POV inputs. The POV angle is used to determine the
     * direction
     *
     * @return the controller inputs as a ChassisSpeeds object, where the x and y components
     *     represent the translation speeds and the omega component represents the angular speed
     */
    private ChassisSpeeds calculateSpeedBasedOnPOVInputs() {
        Optional<Rotation2d> pov = driver_inputs_.pov().get();
        if (pov.isEmpty()) {
            return new ChassisSpeeds();
        }

        // Calculate the x and y magnitudes based on the POV angle
        double x_magnitude = pov.get().getCos();
        double y_magnitude = -pov.get().getSin(); // Negate because WPILib Y+ is left
        double angular_magnitude =
                -MathUtil.applyDeadband(
                        driver_inputs_.right_x().getAsDouble(), CONSTANTS.CONTROLLER_DEADBAND);

        ChassisSpeeds speeds =
                new ChassisSpeeds(
                        x_magnitude * CONSTANTS.MAX_CRAWL_RATE,
                        y_magnitude * CONSTANTS.MAX_CRAWL_RATE,
                        angular_magnitude * CONSTANTS.MAX_ANGULAR_RATE);
        MwLog.log(getSubsystemKey() + "RequestedChassisSpeeds", speeds);
        return speeds;
    }

    /**
     * Sets the operator forward direction based on the operator's perspective.
     *
     * @param reference the operator's perspective reference
     */
    public void setOperatorForwardDirection(OperatorPerspective reference) {
        operator_forward_direction_ = reference.heading;
        MwLog.log(
                getSubsystemKey() + "OperatorForwardDirection",
                operator_forward_direction_.getDegrees(),
                Degrees);
    }

    /**
     * Sets the teleop velocity scalar to scale the robot's speed.
     *
     * @param scalar the scalar value to set, clamped between 0 and 1
     */
    public void setTeleOpVelocityScalar(double scalar) {
        tele_op_velocity_scalar_ = MathUtil.clamp(scalar, 0, 1);
    }

    /**
     * Sets the teleop velocity rate limit scalar. This updates the slew rate limiters dynamically
     * to control acceleration/deceleration smoothness.
     *
     * @param rate_limit_scalar the scalar to apply to the max translation rate for slew limiting
     */
    public void setTeleOpVelocityRateLimitScalar(double rate_limit_scalar) {
        tele_op_velocity_rl_scalar_ = MathUtil.clamp(rate_limit_scalar, 0.0, 1.0);
        // Update the rate limits dynamically without losing state
        x_tele_op_velocity_slew_limiter_.setRateLimit(
                CONSTANTS.MAX_TRANSLATION_ACCEL * tele_op_velocity_rl_scalar_);
        y_tele_op_velocity_slew_limiter_.setRateLimit(
                CONSTANTS.MAX_TRANSLATION_ACCEL * tele_op_velocity_rl_scalar_);
    }

    // ------------------------------------------------
    // Status Check Methods
    // ------------------------------------------------

    /**
     * Checks if the robot is at the tractor beam setpoint.
     *
     * @return true if the robot is at the tractor beam setpoint, false otherwise
     */
    public boolean isAtTractorBeamSetpoint() {
        double distance =
                desired_tractor_beam_pose_
                        .getTranslation()
                        .minus(getFieldPose().getTranslation())
                        .getNorm();
        return MathUtil.isNear(0.0, distance, CONSTANTS.TRACTOR_BEAM_TRANSLATION_ERROR_MARGIN);
    }

    /**
     * Checks if the robot is at the desired rotation.
     *
     * @return true if the robot is at the desired rotation, false otherwise
     */
    public boolean isAtDesiredRotation() {
        return isAtDesiredRotation(Units.degreesToRadians(10.0));
    }

    /**
     * Checks if the robot is at the desired rotation within a specified tolerance.
     *
     * @param tolerance the tolerance in radians
     * @return true if the robot is at the desired rotation within the tolerance, false otherwise
     */
    public boolean isAtDesiredRotation(double tolerance) {
        return Math.abs(field_centric_rotation_lock_request_.HeadingController.getPositionError())
                < tolerance;
    }

    /**
     * Checks if the choreo trajectory time has elapsed.
     *
     * @return true if the choreo trajectory time has elapsed, false otherwise
     */
    public boolean hasChoreoTimeElapsed() {
        if (system_state_ != SwerveStates.CHOREO_PATH
                && system_state_ != SwerveStates.CHOREO_PATH_ROTATION_LOCK) {
            return false;
        }
        return choreo_timer_.get() >= desired_choreo_traj_.getTotalTime();
    }

    /**
     * Checks if the choreo trajectory time has elapsed for a specified total time.
     *
     * @param total_time the total time to check against
     * @return true if the choreo trajectory time has elapsed, false otherwise
     */
    public boolean hasChoreoTimeElapsed(double total_time) {
        if (system_state_ != SwerveStates.CHOREO_PATH
                && system_state_ != SwerveStates.CHOREO_PATH_ROTATION_LOCK) {
            return false;
        }
        return choreo_timer_.get() >= total_time;
    }

    /**
     * Checks if the robot is at the choreo setpoint.
     *
     * @return true if the robot is at the choreo setpoint, false otherwise
     */
    public boolean isAtChoreoSetpoint() {
        if (system_state_ != SwerveStates.CHOREO_PATH
                && system_state_ != SwerveStates.CHOREO_PATH_ROTATION_LOCK) {
            return false;
        }
        return MathUtil.isNear(
                        desired_choreo_traj_.getFinalPose(false).get().getX(),
                        getFieldPose().getX(),
                        CONSTANTS.CHOREO_TRANSLATION_ERROR_MARGIN)
                && MathUtil.isNear(
                        desired_choreo_traj_.getFinalPose(false).get().getY(),
                        getFieldPose().getY(),
                        CONSTANTS.CHOREO_TRANSLATION_ERROR_MARGIN)
                && MathUtil.isNear(
                        desired_choreo_traj_.getFinalSample(false).get().vx,
                        getCurrentChassisSpeedsFieldRelative()
                                .vxMetersPerSecond,
                        CONSTANTS.CHOREO_VELOCITY_ERROR_MARGIN)
                && MathUtil.isNear(
                        desired_choreo_traj_.getFinalSample(false).get().vy,
                        getCurrentChassisSpeedsFieldRelative()
                                .vyMetersPerSecond,
                        CONSTANTS.CHOREO_VELOCITY_ERROR_MARGIN)
                && choreo_timer_.get() >= desired_choreo_traj_.getTotalTime();
    }

    /**
     * Checks if the robot is at the end of the choreo trajectory or at the tractor beam setpoint.
     *
     * @return true if the robot is at the end of the choreo trajectory or at the tractor beam
     *     setpoint, false otherwise
     */
    public boolean isAtEndOfChoreoTrajectoryOrTractorBeam() {
        if (desired_choreo_traj_ != null) {
            return (MathUtil.isNear(
                                    desired_choreo_traj_.getFinalPose(false).get().getX(),
                                    getFieldPose().getX(),
                                    CONSTANTS.CHOREO_TRANSLATION_ERROR_MARGIN))
                            && MathUtil.isNear(
                                    desired_choreo_traj_.getFinalPose(false).get().getY(),
                                    getFieldPose().getY(),
                                    CONSTANTS.CHOREO_TRANSLATION_ERROR_MARGIN)
                    || isAtTractorBeamSetpoint();
        } else {
            return isAtTractorBeamSetpoint();
        }
    }

    /**
     * Gets the distance from the choreo endpoint.
     *
     * @return the distance from the choreo endpoint in meters
     */
    public double getDistanceFromChoreoEndpoint() {
        double distance =
                Math.abs(
                        desired_choreo_traj_
                                .getFinalPose(false)
                                .get()
                                .minus(getFieldPose())
                                .getTranslation()
                                .getNorm());
        return distance;
    }

    /**
     * Gets the distance from the tractor beam setpoint.
     *
     * @return the distance from the tractor beam setpoint in meters
     */
    public double getDistanceFromTractorBeamSetpoint() {
        double diff =
                desired_tractor_beam_pose_
                        .getTranslation()
                        .minus(getFieldPose().getTranslation())
                        .getNorm();
        return diff;
    }

    // ------------------------------------------------
    // Choreo Event Methods
    // ------------------------------------------------

    /**
     * Gets a trigger for the specified Choreo event. The trigger will be true from when the event
     * timestamp is passed until trajectory ends. Use .onTrue() for one-time actions or .whileTrue()
     * for continuous actions.
     *
     * @param event_name The name of the event defined in Choreo
     * @return A Trigger that reads from the event HashMap
     */
    public Trigger getChoreoEventTimeTrigger(String event_name) {
        return choreo_event_tracker_.getTimeTrigger(event_name);
    }

    /**
     * Gets a trigger for the specified Choreo event that checks if the robot is at the event's
     * pose. This is a pose-only check (does not require time condition). The trigger will be true
     * when the robot is within tolerances of the event's pose.
     *
     * @param event_name The name of the event defined in Choreo
     * @param translation_tol The translation distance tolerance in meters
     * @param rotation_tol The rotation tolerance in radians
     * @return A Trigger that checks pose only
     */
    public Trigger getChoreoEventPoseTrigger(
            String event_name, double translation_tol, double rotation_tol) {
        return choreo_event_tracker_.getPoseTrigger(event_name, translation_tol, rotation_tol);
    }

    /**
     * Checks if a specific Choreo event has been passed during trajectory following.
     *
     * @param event_name The name of the event to check
     * @return true if the event has been passed, false otherwise
     */
    public boolean hasChoreoEventBeenPassed(String event_name) {
        return choreo_event_tracker_.hasEventBeenPassed(event_name);
    }

    /**
     * Gets the chassis translation velocity in meters per second by calculating the magnitude of
     * the current chassis speeds.
     *
     * @return the chassis translation velocity in meters per second
     */
    private double getChassisTranslationVelocity() {
        ChassisSpeeds current_speeds = getDesiredChassisSpeeds();
        return Math.hypot(current_speeds.vxMetersPerSecond, current_speeds.vyMetersPerSecond);
    }

    /**
     * Gets the chassis angular velocity in radians per second from the current chassis speeds.
     *
     * @return the chassis angular velocity in radians per second
     */
    private double getChassisAngularVelocity() {
        return swerve_mech_.getCurrentChassisSpeeds().omegaRadiansPerSecond;
    }

    /**
     * Checks if the chassis is stationary by comparing the translation and angular velocities to
     * predefined thresholds.
     *
     * @return true if the chassis is stationary, false otherwise
     */
    public boolean isChassisStationary() {
        return getChassisTranslationVelocity() < CONSTANTS.STATIONARY_TRANSLATION_VELOCITY_THRESHOLD
                && Math.abs(getChassisAngularVelocity())
                        < CONSTANTS.STATIONARY_ANGULAR_VELOCITY_THRESHOLD;
    }

    // ------------------------------------------------
    // Chassis Property Methods
    // ------------------------------------------------

    /** Stores the current encoder readings as offsets */
    public Command setModuleOffsets() {
        return Commands.runOnce(() -> swerve_mech_.setModuleOffsets())
                .withName("Set Module Offsets");
    }

    /** Zeros the gyro yaw to the operator forward direction */
    public Command zeroGyroYaw() {
        return Commands.runOnce(() -> swerve_mech_.setGyroYaw(operator_forward_direction_))
                .withName(
                        "Zero Gyro Yaw: " + operator_forward_direction_.getDegrees() + " Degrees");
    }

    /**
     * Sets the gyro yaw to a specific heading. This is useful for setting the gyro to a known
     * heading during autonomous or if the gyro drifts significantly during a match.
     *
     * @param yaw the desired heading to set the gyro to
     */
    public void setGyroYaw(Rotation2d yaw) {
        swerve_mech_.setGyroYaw(yaw);
    }

    /**
     * Returns the current module states (turn angles and drive velocities) for all of the modules.
     */
    public SwerveModuleState[] getCurrentModuleStates() {
        return swerve_mech_.getCurrentModuleStates();
    }

    /**
     * Returns the setpoint module states (turn angles and drive velocities) for all of the modules.
     */
    public SwerveModuleState[] getSetpointModuleStates() {
        return swerve_mech_.getSetpointModuleStates();
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    public SwerveModulePosition[] getCurrentModulePositions() {
        return swerve_mech_.getModulePositions();
    }

    /** Returns the measured chassis speeds of the robot. */
    public ChassisSpeeds getCurrentChassisSpeeds() {
        return swerve_mech_.getCurrentChassisSpeeds();
    }

    /** Returns the setpoint chassis speeds of the robot. */
    public ChassisSpeeds getSetpointChassisSpeeds() {
        return swerve_mech_.getSetpointChassisSpeeds();
    }

    /** Returns the desired chassis speeds of the robot (target speeds from commands). */
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return desired_chassis_speeds_;
    }

    /** Returns the raw gyro rotation */
    public Rotation2d getGyroYaw() {
        return swerve_mech_.getGyroYaw();
    }

    /** Returns the raw gyro yaw rate */
    public double getGyroYawRate() {
        return swerve_mech_.getGyroYawRate();
    }

    /**
     * Returns the swerve drive kinematics instance.
     *
     * @return SwerveDriveKinematics object representing the swerve drive
     */
    public SwerveDriveKinematics getKinematics() {
        return swerve_mech_.getKinematics();
    }

    /** Returns the robot's field pose from the pose supplier given at construction. */
    public Pose2d getFieldPose() {
        return pose_supplier_.get();
    }

    /** Returns the measured chassis speeds rotated into the field frame. */
    public ChassisSpeeds getCurrentChassisSpeedsFieldRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                getCurrentChassisSpeeds(), getFieldPose().getRotation());
    }

    /** Returns the underlying swerve mechanism for subclasses that need direct access. */
    protected SwerveMech getSwerveMech() {
        return swerve_mech_;
    }

    private ChassisSpeeds removeOperatorPerspective(ChassisSpeeds speeds) {
        Translation2d tmp = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        tmp = tmp.rotateBy(operator_forward_direction_);
        return new ChassisSpeeds(tmp.getX(), tmp.getY(), speeds.omegaRadiansPerSecond);
    }
}
