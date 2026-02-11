package com.marswars.swerve_lib;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.marswars.swerve_lib.module.Module;

/**
 * Container for all the Swerve Requests. Use this to find all applicable swerve drive requests.
 *
 * <p>This is also an interface common to all swerve drive control requests that allow the request
 * to calculate the state to apply to the modules.
 */
public interface ChassisRequest {

    /**
     * The reference for "forward" is sometimes different if you're talking about field relative.
     * This addresses which forward to use.
     */
    public enum XPositiveReference {
        /**
         * This forward reference makes it so "forward" (positive X) is always towards the red
         * alliance. This is important in situations such as path following where positive X is
         * always towards the red alliance wall, regardless of where the operator physically are
         * located.
         */
        TowardsRedAlliance,
        /**
         * This forward references makes it so "forward" (positive X) is determined from the
         * operator's perspective. This is important for most teleop driven field-centric requests,
         * where positive X really means to drive away from the operator.
         *
         * <p><b>Important</b>: Users must specify the OperatorPerspective with {@link
         * SwerveDrivetrain} object
         */
        OperatorPerspective
    }

    /** Contains everything the control requests need to calculate the module state. */
    public class ChassisRequestParameters {
        public SwerveDriveKinematics kinematics;
        public ChassisSpeeds currentChassisSpeed;
        public Pose2d currentPose;
        public double timestamp;
        public Translation2d[] moduleLocations;
        public Rotation2d operatorForwardDirection;
        public double updatePeriod;
    }

    /**
     * Applies this swerve request to the given modules. This is typically called by the
     * SwerveDrivetrain.
     *
     * @param parameters Parameters the control request needs to calculate the module state
     * @param modulesToApply Modules to which the control request is applied
     */
    public void apply(ChassisRequestParameters parameters, Module... modulesToApply);

    /**
     * Sets the swerve drive module states to point inward on the robot in an "X" fashion, creating
     * a natural brake which will oppose any motion.
     */
    public class SwerveDriveBrake implements ChassisRequest {

        /** The type of control request to use for the drive motor. */
        public Module.DriveControlMode DriveRequestType = Module.DriveControlMode.OPEN_LOOP;

        /** The type of control request to use for the steer motor. */
        public Module.SteerControlMode SteerRequestType = Module.SteerControlMode.CLOSED_LOOP;

    /** {@inheritDoc} */
    public void apply(ChassisRequestParameters parameters, Module... modulesToApply) {

            for (int i = 0; i < modulesToApply.length; ++i) {
                SwerveModuleState state =
                        new SwerveModuleState(0, parameters.moduleLocations[i].getAngle());
                modulesToApply[i].runSetpoint(state, DriveRequestType, SteerRequestType);
            }
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public SwerveDriveBrake withDriveRequestType(Module.DriveControlMode driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public SwerveDriveBrake withSteerRequestType(Module.SteerControlMode steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }

    /**
     * Drives the swerve drivetrain in a field-centric manner.
     *
     * <p>When users use this request, they specify the direction the robot should travel oriented
     * against the field, and the rate at which their robot should rotate about the center of the
     * robot.
     */
    public class FieldCentric implements ChassisRequest {

        /** The desired x/y and rotation rate */
        public Twist2d Twist = new Twist2d();

        /** The allowable deadband of the request. */
        public double Deadband = 0;

        /** The rotational deadband of the request. */
        public double RotationalDeadband = 0;

        /**
         * The center of rotation the robot should rotate around. This is (0,0) by default, which
         * will rotate around the center of the robot.
         */
        public Translation2d CenterOfRotation = new Translation2d();

        /** The type of control request to use for the drive motor. */
        public Module.DriveControlMode DriveRequestType = Module.DriveControlMode.OPEN_LOOP;

        /** The type of control request to use for the steer motor. */
        public Module.SteerControlMode SteerRequestType = Module.SteerControlMode.CLOSED_LOOP;

        /** The perspective to use when determining which direction is forward. */
        public XPositiveReference XPositiveReference =
                ChassisRequest.XPositiveReference.OperatorPerspective;

        /** The last applied state in case we don't have anything to drive. */
        protected SwerveModuleState[] m_lastAppliedState = null;

    /** {@inheritDoc} */
    public void apply(ChassisRequestParameters parameters, Module... modulesToApply) {
            double toApplyX = Twist.dx;
            double toApplyY = Twist.dy;
            if (XPositiveReference == ChassisRequest.XPositiveReference.OperatorPerspective) {
                /* If we're operator perspective, modify the X/Y translation by the angle */
                Translation2d tmp = new Translation2d(toApplyX, toApplyY);
                tmp = tmp.rotateBy(parameters.operatorForwardDirection);
                toApplyX = tmp.getX();
                toApplyY = tmp.getY();
            }
            double toApplyOmega = Twist.dtheta;
            if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
                toApplyX = 0;
                toApplyY = 0;
            }
            if (Math.abs(toApplyOmega) < RotationalDeadband) {
                toApplyOmega = 0;
            }

            ChassisSpeeds speeds =
                    ChassisSpeeds.discretize(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    toApplyX,
                                    toApplyY,
                                    toApplyOmega,
                                    parameters.currentPose.getRotation()),
                            parameters.updatePeriod);

            var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].runSetpoint(states[i], DriveRequestType, SteerRequestType);
            }
        }

        /**
         * The linear and angular velocity to apply to the drivetrain.
         *
         * @param twist x/y and rotation rate to apply
         * @return this request
         */
        public FieldCentric withTwist(Twist2d twist) {
            this.Twist = twist;
            return this;
        }

        /**
         * Sets the allowable deadband of the request.
         *
         * @param deadband Allowable deadband of the request
         * @return this request
         */
        public FieldCentric withDeadband(double deadband) {
            this.Deadband = deadband;
            return this;
        }

        /**
         * Sets the rotational deadband of the request.
         *
         * @param rotationalDeadband Rotational deadband of the request
         * @return this request
         */
        public FieldCentric withRotationalDeadband(double rotationalDeadband) {
            this.RotationalDeadband = rotationalDeadband;
            return this;
        }

        /**
         * Sets the center of rotation of the request
         *
         * @param centerOfRotation The center of rotation the robot should rotate around.
         * @return this request
         */
        public FieldCentric withCenterOfRotation(Translation2d centerOfRotation) {
            this.CenterOfRotation = centerOfRotation;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public FieldCentric withDriveRequestType(Module.DriveControlMode driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public FieldCentric withSteerRequestType(Module.SteerControlMode steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }

        /**
         * Sets the perspective to use when determining which direction is forward.
         *
         * @param xPositiveReference The perspective to use when determining which direction is
         *     forward.
         * @return this request
         */
        public FieldCentric withXPositiveReference(XPositiveReference xPositiveReference) {
            this.XPositiveReference = xPositiveReference;
            return this;
        }
    }

    /**
     * Drives the swerve drivetrain in a field-centric manner, maintaining a specified heading angle
     * to ensure the robot is facing the desired direction
     *
     * <p>When users use this request, they specify the direction the robot should travel oriented
     * against the field, and the direction the robot should be facing.
     *
     * <p>This control request is especially useful for autonomous control, where the robot should
     * be facing a changing direction throughout the motion.
     */
    public class FieldCentricFacingAngle implements ChassisRequest {

        /** The desired x/y and rotation rate */
        public Twist2d Twist = new Twist2d();

        /**
         * The direction the robot should face. 0 Degrees is defined as in the direction of the X
         * axis.
         */
        public Rotation2d TargetDirection = new Rotation2d();

        /** The allowable deadband of the request. */
        public double Deadband = 0;

        /** The rotational deadband of the request. */
        public double RotationalDeadband = 0;

        /** The maximum absolute rotational rate of the request. */
        public double MaxAbsRotationalRate = 0;

        /**
         * The center of rotation the robot should rotate around. This is (0,0) by default, which
         * will rotate around the center of the robot.
         */
        public Translation2d CenterOfRotation = new Translation2d();

        /** The type of control request to use for the drive motor. */
        public Module.DriveControlMode DriveRequestType = Module.DriveControlMode.OPEN_LOOP;

        /** The type of control request to use for the steer motor. */
        public Module.SteerControlMode SteerRequestType = Module.SteerControlMode.CLOSED_LOOP;

        /**
         * The PID controller used to maintain the desired heading. Users can specify the PID gains
         * to change how aggressively to maintain heading.
         *
         * <p>This PID controller operates on heading radians and outputs a target rotational rate
         * in radians per second.
         */
        public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0.0);

        /** The perspective to use when determining which direction is forward. */
        public XPositiveReference XPositiveReference =
                ChassisRequest.XPositiveReference.OperatorPerspective;

    /** {@inheritDoc} */
    public void apply(ChassisRequestParameters parameters, Module... modulesToApply) {
            double toApplyX = Twist.dx;
            double toApplyY = Twist.dy;
            Rotation2d angleToFace = TargetDirection;
            HeadingController.enableContinuousInput(0, 2 * Math.PI);
            if (XPositiveReference == ChassisRequest.XPositiveReference.OperatorPerspective) {
                /* If we're operator perspective, modify the X/Y translation by the angle */
                Translation2d tmp = new Translation2d(toApplyX, toApplyY);
                tmp = tmp.rotateBy(parameters.operatorForwardDirection);
                toApplyX = tmp.getX();
                toApplyY = tmp.getY();
                /* And rotate the direction we want to face by the angle */
                // angleToFace = angleToFace.rotateBy(parameters.operatorForwardDirection);
            }

            Pose2d currentRobotPose = parameters.currentPose;
            double rotationRate =
                    HeadingController.calculate(
                            currentRobotPose.getRotation().getRadians(),
                            angleToFace.getRadians(),
                            parameters.timestamp);

            double toApplyOmega = rotationRate;
            if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
                toApplyX = 0;
                toApplyY = 0;
            }
            if (Math.abs(toApplyOmega) < RotationalDeadband) {
                toApplyOmega = 0;
            }
            if (MaxAbsRotationalRate > 0) {
                toApplyOmega =
                        MathUtil.clamp(toApplyOmega, -MaxAbsRotationalRate, MaxAbsRotationalRate);
            }

            ChassisSpeeds speeds =
                    ChassisSpeeds.discretize(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    toApplyX,
                                    toApplyY,
                                    toApplyOmega,
                                    parameters.currentPose.getRotation()),
                            parameters.updatePeriod);

            var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].runSetpoint(states[i], DriveRequestType, SteerRequestType);
            }
        }

        /**
         * The linear and angular velocity to apply to the drivetrain.
         *
         * @param twist x/y and rotation rate to apply
         * @return this request
         */
        public FieldCentricFacingAngle withTwist(Twist2d twist) {
            this.Twist = twist;
            return this;
        }

        /**
         * Sets the desired direction to face. 0 Degrees is defined as in the direction of the X
         * axis. As a result, a TargetDirection of 90 degrees will point along the Y axis, or to the
         * left.
         *
         * @param targetDirection Desired direction to face
         * @return this request
         */
        public FieldCentricFacingAngle withTargetHeading(Rotation2d targetDirection) {
            this.TargetDirection = targetDirection;
            return this;
        }

        /**
         * Sets the allowable deadband of the request.
         *
         * @param deadband Allowable deadband of the request
         * @return this request
         */
        public FieldCentricFacingAngle withDeadband(double deadband) {
            this.Deadband = deadband;
            return this;
        }

        /**
         * Sets the rotational deadband of the request.
         *
         * @param rotationalDeadband Rotational deadband of the request
         * @return this request
         */
        public FieldCentricFacingAngle withRotationalDeadband(double rotationalDeadband) {
            this.RotationalDeadband = rotationalDeadband;
            return this;
        }

        /**
         * Sets the maximum absolute rotational rate of the request.
         *
         * @param maxAbsRotationalRate The maximum absolute rotational rate of the
         * @return this request
         */
        public FieldCentricFacingAngle withMaxAbsRotationalRate(double maxAbsRotationalRate) {
            this.MaxAbsRotationalRate = maxAbsRotationalRate;
            return this;
        }

        /**
         * Sets the center of rotation of the request
         *
         * @param centerOfRotation The center of rotation the robot should rotate around.
         * @return this request
         */
        public FieldCentricFacingAngle withCenterOfRotation(Translation2d centerOfRotation) {
            this.CenterOfRotation = centerOfRotation;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public FieldCentricFacingAngle withDriveRequestType(
                Module.DriveControlMode driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public FieldCentricFacingAngle withSteerRequestType(
                Module.SteerControlMode steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }

        /**
         * Sets the perspective to use when determining which direction is forward.
         *
         * @param xPositiveReference The perspective to use when determining which direction is
         *     forward.
         * @return this request
         */
        public FieldCentricFacingAngle withXPositiveReference(
                XPositiveReference xPositiveReference) {
            this.XPositiveReference = xPositiveReference;
            return this;
        }

        /**
         * Sets the PID controller used to maintain the desired heading. Users can specify the PID
         * gains to change how aggressively to maintain heading.
         *
         * @param headingController The PID controller used to maintain the desired heading.
         * @return this request
         */
        public FieldCentricFacingAngle withHeadingController(
                PhoenixPIDController headingController) {
            this.HeadingController = headingController;
            return this;
        }
    }

    /**
     * Does nothing to the swerve module state. This is the default state of a newly created swerve
     * drive mechanism.
     */
    public class Idle implements ChassisRequest {
    /** {@inheritDoc} */
    public void apply(ChassisRequestParameters parameters, Module... modulesToApply) {}
    }

    /** Sets the swerve drive modules to point to a specified direction. */
    public class PointWheelsAt implements ChassisRequest {

        /**
         * The direction to point the modules toward. This direction is still optimized to what the
         * module was previously at.
         */
        public Rotation2d ModuleDirection = new Rotation2d();

        /** The type of control request to use for the drive motor. */
        public Module.DriveControlMode DriveRequestType = Module.DriveControlMode.OPEN_LOOP;

        /** The type of control request to use for the steer motor. */
        public Module.SteerControlMode SteerRequestType = Module.SteerControlMode.CLOSED_LOOP;

    /** {@inheritDoc} */
    public void apply(ChassisRequestParameters parameters, Module... modulesToApply) {

            for (int i = 0; i < modulesToApply.length; ++i) {
                SwerveModuleState state = new SwerveModuleState(0, ModuleDirection);
                modulesToApply[i].runSetpoint(state, DriveRequestType, SteerRequestType);
            }
        }

        /**
         * Sets the direction to point the modules toward. This direction is still optimized to what
         * the module was previously at.
         *
         * @param moduleDirection Direction to point the modules toward
         * @return this request
         */
        public PointWheelsAt withModuleDirection(Rotation2d moduleDirection) {
            this.ModuleDirection = moduleDirection;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public PointWheelsAt withDriveRequestType(Module.DriveControlMode driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public PointWheelsAt withSteerRequestType(Module.SteerControlMode steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }

    /**
     * Drives the swerve drivetrain in a robot-centric manner.
     *
     * <p>When users use this request, they specify the direction the robot should travel oriented
     * against the robot itself, and the rate at which their robot should rotate about the center of
     * the robot.
     *
     * <p>An example scenario is that the robot is oriented to the east, the VelocityX is +5 m/s,
     * VelocityY is 0 m/s, and RotationRate is 0.5 rad/s. In this scenario, the robot would drive
     * eastward at 5 m/s and turn counterclockwise at 0.5 rad/s.
     */
    public class RobotCentric implements ChassisRequest {

        /** The desired x/y and rotation rate */
        public Twist2d Twist = new Twist2d();

        /** The allowable deadband of the request. */
        public double Deadband = 0;

        /** The rotational deadband of the request. */
        public double RotationalDeadband = 0;

        /**
         * The center of rotation the robot should rotate around. This is (0,0) by default, which
         * will rotate around the center of the robot.
         */
        public Translation2d CenterOfRotation = new Translation2d();

        /** The type of control request to use for the drive motor. */
        public Module.DriveControlMode DriveRequestType = Module.DriveControlMode.OPEN_LOOP;

        /** The type of control request to use for the steer motor. */
        public Module.SteerControlMode SteerRequestType = Module.SteerControlMode.CLOSED_LOOP;

    /** {@inheritDoc} */
    public void apply(ChassisRequestParameters parameters, Module... modulesToApply) {
            double toApplyX = Twist.dx;
            double toApplyY = Twist.dy;
            double toApplyOmega = Twist.dtheta;
            if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
                toApplyX = 0;
                toApplyY = 0;
            }
            if (Math.abs(toApplyOmega) < RotationalDeadband) {
                toApplyOmega = 0;
            }
            ChassisSpeeds speeds = new ChassisSpeeds(toApplyX, toApplyY, toApplyOmega);

            var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].runSetpoint(states[i], DriveRequestType, SteerRequestType);
            }
        }

        /**
         * The linear and angular velocity to apply to the drivetrain.
         *
         * @param twist x/y and rotation rate to apply
         * @return this request
         */
        public RobotCentric withTwist(Twist2d twist) {
            this.Twist = twist;
            return this;
        }

        /**
         * Sets the allowable deadband of the request.
         *
         * @param deadband Allowable deadband of the request
         * @return this request
         */
        public RobotCentric withDeadband(double deadband) {
            this.Deadband = deadband;
            return this;
        }

        /**
         * Sets the rotational deadband of the request.
         *
         * @param rotationalDeadband Rotational deadband of the request
         * @return this request
         */
        public RobotCentric withRotationalDeadband(double rotationalDeadband) {
            this.RotationalDeadband = rotationalDeadband;
            return this;
        }

        /**
         * Sets the center of rotation of the request
         *
         * @param centerOfRotation The center of rotation the robot should rotate around.
         * @return this request
         */
        public RobotCentric withCenterOfRotation(Translation2d centerOfRotation) {
            this.CenterOfRotation = centerOfRotation;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public RobotCentric withDriveRequestType(Module.DriveControlMode driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public RobotCentric withSteerRequestType(Module.SteerControlMode steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }

    /** Accepts a generic ChassisSpeeds to apply to the drivetrain. */
    public class ApplyChassisSpeeds implements ChassisRequest {

        /** The chassis speeds to apply to the drivetrain. */
        public ChassisSpeeds Speeds = new ChassisSpeeds();

        /** The center of rotation to rotate around. */
        public Translation2d CenterOfRotation = new Translation2d(0, 0);

        /** The type of control request to use for the drive motor. */
        public Module.DriveControlMode DriveRequestType = Module.DriveControlMode.OPEN_LOOP;

        /** The type of control request to use for the steer motor. */
        public Module.SteerControlMode SteerRequestType = Module.SteerControlMode.CLOSED_LOOP;

    /** {@inheritDoc} */
    public void apply(ChassisRequestParameters parameters, Module... modulesToApply) {
            var states = parameters.kinematics.toSwerveModuleStates(Speeds, CenterOfRotation);
            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].runSetpoint(states[i], DriveRequestType, SteerRequestType);
            }
        }

        /**
         * Sets the chassis speeds to apply to the drivetrain.
         *
         * @param speeds Chassis speeds to apply to the drivetrain
         * @return this request
         */
        public ApplyChassisSpeeds withSpeeds(ChassisSpeeds speeds) {
            this.Speeds = speeds;
            return this;
        }

        /**
         * Sets the center of rotation to rotate around.
         *
         * @param centerOfRotation Center of rotation to rotate around
         * @return this request
         */
        public ApplyChassisSpeeds withCenterOfRotation(Translation2d centerOfRotation) {
            this.CenterOfRotation = centerOfRotation;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public ApplyChassisSpeeds withDriveRequestType(Module.DriveControlMode driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public ApplyChassisSpeeds withSteerRequestType(Module.SteerControlMode steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }

    /** Accepts a generic Field Relative ChassisSpeeds to apply to the drivetrain. */
    public class ApplyFieldSpeeds implements ChassisRequest {

        /** The chassis speeds to apply to the drivetrain. */
        public ChassisSpeeds Speeds = new ChassisSpeeds();

        /**
         * The center of rotation the robot should rotate around. This is (0,0) by default, which
         * will rotate around the center of the robot.
         */
        public Translation2d CenterOfRotation = new Translation2d();

        /** The type of control request to use for the drive motor. */
        public Module.DriveControlMode DriveRequestType = Module.DriveControlMode.OPEN_LOOP;

        /** The type of control request to use for the steer motor. */
        public Module.SteerControlMode SteerRequestType = Module.SteerControlMode.CLOSED_LOOP;

    /** {@inheritDoc} */
    public void apply(ChassisRequestParameters parameters, Module... modulesToApply) {
            ChassisSpeeds speeds =
                    ChassisSpeeds.discretize(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    Speeds, parameters.currentPose.getRotation()),
                            parameters.updatePeriod);

            var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].runSetpoint(states[i], DriveRequestType, SteerRequestType);
            }
        }

        /**
         * Sets the chassis speeds to apply to the drivetrain.
         *
         * @param speeds Chassis speeds to apply to the drivetrain
         * @return this request
         */
        public ApplyFieldSpeeds withSpeeds(ChassisSpeeds speeds) {
            this.Speeds = speeds;
            return this;
        }

        /**
         * Sets the center of rotation of the request
         *
         * @param centerOfRotation The center of rotation the robot should rotate around.
         * @return this request
         */
        public ApplyFieldSpeeds withCenterOfRotation(Translation2d centerOfRotation) {
            this.CenterOfRotation = centerOfRotation;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public ApplyFieldSpeeds withDriveRequestType(Module.DriveControlMode driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public ApplyFieldSpeeds withSteerRequestType(Module.SteerControlMode steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }

    /**
     * Applies robot-relative chassis speeds while maintaining a field-centric facing angle.
     * This allows the robot to move in any direction relative to itself while independently
     * controlling its orientation to face a specific field-relative direction.
     */
    public class RobotCentricFacingAngle implements ChassisRequest {

        /** The robot-relative chassis speeds to apply to the drivetrain. */
        public ChassisSpeeds Speeds = new ChassisSpeeds();

        /**
         * The direction the robot should face. 0 Degrees is defined as in the direction of the X
         * axis.
         */
        public Rotation2d TargetDirection = new Rotation2d();

        /** The allowable deadband of the request. */
        public double Deadband = 0;

        /** The rotational deadband of the request. */
        public double RotationalDeadband = 0;

        /** The maximum absolute rotational rate of the request. */
        public double MaxAbsRotationalRate = 0;

        /**
         * The center of rotation the robot should rotate around. This is (0,0) by default, which
         * will rotate around the center of the robot.
         */
        public Translation2d CenterOfRotation = new Translation2d();

        /** The type of control request to use for the drive motor. */
        public Module.DriveControlMode DriveRequestType = Module.DriveControlMode.OPEN_LOOP;

        /** The type of control request to use for the steer motor. */
        public Module.SteerControlMode SteerRequestType = Module.SteerControlMode.CLOSED_LOOP;

        /**
         * The PID controller used to maintain the desired heading. Users can specify the PID gains
         * to change how aggressively to maintain heading.
         *
         * <p>This PID controller operates on heading radians and outputs a target rotational rate
         * in radians per second.
         */
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0.0);

    /** {@inheritDoc} */
    public void apply(ChassisRequestParameters parameters, Module... modulesToApply) {
            // Get the robot-relative speeds
            double toApplyVx = Speeds.vxMetersPerSecond;
            double toApplyVy = Speeds.vyMetersPerSecond;
            if (Math.sqrt(toApplyVx * toApplyVx + toApplyVy * toApplyVy) < Deadband) {
                toApplyVx = 0;
                toApplyVy = 0;
            }
            // Calculate rotational rate to maintain target heading
            Rotation2d angleToFace = TargetDirection;
            HeadingController.enableContinuousInput(0, 2 * Math.PI);
            
            Pose2d currentRobotPose = parameters.currentPose;
            double rotationRate =
                    HeadingController.calculate(
                            currentRobotPose.getRotation().getRadians(),
                            angleToFace.getRadians(),
                            parameters.timestamp);

            double toApplyOmega = rotationRate;
            if (Math.abs(toApplyOmega) < RotationalDeadband) {
                toApplyOmega = 0;
            }
            if (MaxAbsRotationalRate > 0) {
                toApplyOmega =
                        MathUtil.clamp(toApplyOmega, -MaxAbsRotationalRate, MaxAbsRotationalRate);
            }

            // Create chassis speeds with robot-relative translation and calculated rotation
            ChassisSpeeds speeds =
                    ChassisSpeeds.discretize(
                            new ChassisSpeeds(toApplyVx, toApplyVy, toApplyOmega),
                            parameters.updatePeriod);

            var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].runSetpoint(states[i], DriveRequestType, SteerRequestType);
            }
        }

        /**
         * Sets the robot-relative chassis speeds to apply to the drivetrain.
         *
         * @param speeds Robot-relative chassis speeds to apply
         * @return this request
         */
        public RobotCentricFacingAngle withSpeeds(ChassisSpeeds speeds) {
            this.Speeds = speeds;
            return this;
        }

        /**
         * Sets the desired direction to face. 0 Degrees is defined as in the direction of the X
         * axis. As a result, a TargetDirection of 90 degrees will point along the Y axis, or to the
         * left.
         *
         * @param targetDirection Desired direction to face
         * @return this request
         */
        public RobotCentricFacingAngle withTargetHeading(Rotation2d targetDirection) {
            this.TargetDirection = targetDirection;
            return this;
        }
        /**
         * Sets the allowable deadband of the request.
         *
         * @param deadband Allowable deadband of the request
         * @return this request
         */
        public RobotCentricFacingAngle withDeadband(double deadband) {
            this.Deadband = deadband;
            return this;
        }
        /**
         * Sets the rotational deadband of the request.
         *
         * @param rotationalDeadband Rotational deadband of the request
         * @return this request
         */
        public RobotCentricFacingAngle withRotationalDeadband(double rotationalDeadband) {
            this.RotationalDeadband = rotationalDeadband;
            return this;
        }

        /**
         * Sets the maximum absolute rotational rate of the request.
         *
         * @param maxAbsRotationalRate The maximum absolute rotational rate
         * @return this request
         */
        public RobotCentricFacingAngle withMaxAbsRotationalRate(double maxAbsRotationalRate) {
            this.MaxAbsRotationalRate = maxAbsRotationalRate;
            return this;
        }

        /**
         * Sets the center of rotation of the request
         *
         * @param centerOfRotation The center of rotation the robot should rotate around.
         * @return this request
         */
        public RobotCentricFacingAngle withCenterOfRotation(Translation2d centerOfRotation) {
            this.CenterOfRotation = centerOfRotation;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public RobotCentricFacingAngle withDriveRequestType(
                Module.DriveControlMode driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public RobotCentricFacingAngle withSteerRequestType(
                Module.SteerControlMode steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }

        /**
         * Sets the PID controller used to maintain the desired heading. Users can specify the PID
         * gains to change how aggressively to maintain heading.
         *
         * @param headingController The PID controller used to maintain the desired heading.
         * @return this request
         */
        public RobotCentricFacingAngle withHeadingController(
                PhoenixPIDController headingController) {
            this.HeadingController = headingController;
            return this;
        }
    }
}
