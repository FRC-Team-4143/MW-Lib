package com.marswars.swerve_lib;

import com.marswars.subsystem.MwConstants;
import edu.wpi.first.math.util.Units;

/**
 * Base constants for {@link MwSwerveSubsystem}. A robot's {@code SwerveConstants} extends this and
 * provides the hardware layout through {@link #getDriveConfig()}. The tunables below are non-final
 * so robot variants can reassign them in their constructors, e.g.
 *
 * <pre>{@code
 * public BetaSwerveConstants() {
 *     MAX_TRANSLATION_RATE = 4.5;
 * }
 * }</pre>
 *
 * <p>Everything is read once, when the subsystem is constructed, so variant constructor values
 * always take effect.
 */
public abstract class MwSwerveConstants extends MwConstants {

    // =============================================================================
    // TELEOP
    // =============================================================================

    /** Deadband applied to raw joystick inputs to prevent drift. */
    public double CONTROLLER_DEADBAND = 0.1;
    /** Max teleop translation speed in meters per second. */
    public double MAX_TRANSLATION_RATE = 5.0;
    /** Max teleop translation acceleration in m/s^2 (slew rate limiters). */
    public double MAX_TRANSLATION_ACCEL = 40.0;
    /** Max speed during crawl (POV) driving in meters per second. */
    public double MAX_CRAWL_RATE = 0.5;
    /** Max angular rate in radians per second. */
    public double MAX_ANGULAR_RATE = 10.0;

    // =============================================================================
    // ROTATION LOCK (shared heading controller)
    // =============================================================================

    public double HEADING_CONTROLLER_KP = 10.0;
    public double HEADING_CONTROLLER_KI = 0.0;
    public double HEADING_CONTROLLER_KD = 1.0;

    // =============================================================================
    // STATIONARY DETECTION
    // =============================================================================

    /** Max translation velocity (m/s) to be considered stationary. */
    public double STATIONARY_TRANSLATION_VELOCITY_THRESHOLD = 0.1;
    /** Max angular velocity (rad/s) to be considered stationary. */
    public double STATIONARY_ANGULAR_VELOCITY_THRESHOLD = 0.2;

    // =============================================================================
    // CHOREO PATH FOLLOWING
    // =============================================================================

    public double CHOREO_TRANSLATION_ERROR_MARGIN = Units.inchesToMeters(1.0);
    public double CHOREO_VELOCITY_ERROR_MARGIN = 0.2;
    public double CHOREO_TRANSLATION_CONTROLLER_KP = 7.0;
    public double CHOREO_TRANSLATION_CONTROLLER_KI = 0.0;
    public double CHOREO_TRANSLATION_CONTROLLER_KD = 0.0;
    public double CHOREO_THETA_CONTROLLER_KP = 12.0;
    public double CHOREO_THETA_CONTROLLER_KI = 0.0;
    public double CHOREO_THETA_CONTROLLER_KD = 1.0;
    /** Distance (m) the robot may lag the trajectory sample before the timer pauses. */
    public double CHOREO_LOOK_AHEAD = 1.0;

    // =============================================================================
    // TRACTOR BEAM
    // =============================================================================

    public double TRACTOR_BEAM_TRANSLATION_ERROR_MARGIN = Units.inchesToMeters(0.5);
    /** Fraction of MAX_TRANSLATION_RATE added to overcome static friction near the target. */
    public double TRACTOR_BEAM_STATIC_FRICTION_CONSTANT = 0.1;
    public double TRACTOR_BEAM_CONTROLLER_KP = 0.0;
    public double TRACTOR_BEAM_CONTROLLER_KI = 0.0;
    public double TRACTOR_BEAM_CONTROLLER_KD = 0.0;

    // =============================================================================
    // HARDWARE
    // =============================================================================

    /**
     * The drivetrain hardware layout (modules, gyro, skid threshold). Called once when the
     * subsystem is constructed, after the constants object (including any variant subclass) is
     * fully constructed; build the config from the fields here, not in a constructor.
     */
    public abstract SwerveDriveConfig getDriveConfig();
}
