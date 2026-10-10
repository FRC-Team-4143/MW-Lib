package com.marswars.swerve_lib;

import com.marswars.subsystem.MwConstants;
import edu.wpi.first.math.util.Units;

/**
 * Base constants for {@link MwSwerveSubsystem}. Every field is final and set once through the
 * constructor: the drivetrain hardware as a {@link SwerveDriveConfig}, plus optional {@link
 * Tuning} overrides (defaults apply otherwise).
 *
 * <p>Per-robot variants pass their values up through {@code super(...)}, e.g. a base robot
 * constants class exposes its hardware builder and a variant subclass edits it:
 *
 * <pre>{@code
 * public BetaSwerveConstants() {
 *     super(alphaHardware().wheelRadius(Units.inchesToMeters(1.978)));
 * }
 * }</pre>
 */
public abstract class MwSwerveConstants extends MwConstants {

    /** The drivetrain hardware layout (modules, gyro, skid threshold). */
    public final SwerveDriveConfig DRIVE_CONFIG;

    // Teleop
    public final double CONTROLLER_DEADBAND;
    public final double MAX_TRANSLATION_RATE;
    public final double MAX_TRANSLATION_ACCEL;
    public final double MAX_CRAWL_RATE;
    public final double MAX_ANGULAR_RATE;

    // Rotation lock (shared heading controller)
    public final double HEADING_CONTROLLER_KP;
    public final double HEADING_CONTROLLER_KI;
    public final double HEADING_CONTROLLER_KD;

    // Stationary detection
    public final double STATIONARY_TRANSLATION_VELOCITY_THRESHOLD;
    public final double STATIONARY_ANGULAR_VELOCITY_THRESHOLD;

    // Choreo path following
    public final double CHOREO_TRANSLATION_ERROR_MARGIN;
    public final double CHOREO_VELOCITY_ERROR_MARGIN;
    public final double CHOREO_TRANSLATION_CONTROLLER_KP;
    public final double CHOREO_TRANSLATION_CONTROLLER_KI;
    public final double CHOREO_TRANSLATION_CONTROLLER_KD;
    public final double CHOREO_THETA_CONTROLLER_KP;
    public final double CHOREO_THETA_CONTROLLER_KI;
    public final double CHOREO_THETA_CONTROLLER_KD;
    public final double CHOREO_LOOK_AHEAD;

    // Tractor beam
    public final double TRACTOR_BEAM_TRANSLATION_ERROR_MARGIN;
    public final double TRACTOR_BEAM_STATIC_FRICTION_CONSTANT;
    public final double TRACTOR_BEAM_CONTROLLER_KP;
    public final double TRACTOR_BEAM_CONTROLLER_KI;
    public final double TRACTOR_BEAM_CONTROLLER_KD;

    /** Constants with the default {@link Tuning}. */
    protected MwSwerveConstants(SwerveDriveConfig drive_config) {
        this(drive_config, new Tuning());
    }

    protected MwSwerveConstants(SwerveDriveConfig drive_config, Tuning tuning) {
        DRIVE_CONFIG = drive_config;

        CONTROLLER_DEADBAND = tuning.controller_deadband;
        MAX_TRANSLATION_RATE = tuning.max_translation_rate;
        MAX_TRANSLATION_ACCEL = tuning.max_translation_accel;
        MAX_CRAWL_RATE = tuning.max_crawl_rate;
        MAX_ANGULAR_RATE = tuning.max_angular_rate;

        HEADING_CONTROLLER_KP = tuning.heading_kp;
        HEADING_CONTROLLER_KI = tuning.heading_ki;
        HEADING_CONTROLLER_KD = tuning.heading_kd;

        STATIONARY_TRANSLATION_VELOCITY_THRESHOLD = tuning.stationary_translation_velocity;
        STATIONARY_ANGULAR_VELOCITY_THRESHOLD = tuning.stationary_angular_velocity;

        CHOREO_TRANSLATION_ERROR_MARGIN = tuning.choreo_translation_error_margin;
        CHOREO_VELOCITY_ERROR_MARGIN = tuning.choreo_velocity_error_margin;
        CHOREO_TRANSLATION_CONTROLLER_KP = tuning.choreo_translation_kp;
        CHOREO_TRANSLATION_CONTROLLER_KI = tuning.choreo_translation_ki;
        CHOREO_TRANSLATION_CONTROLLER_KD = tuning.choreo_translation_kd;
        CHOREO_THETA_CONTROLLER_KP = tuning.choreo_theta_kp;
        CHOREO_THETA_CONTROLLER_KI = tuning.choreo_theta_ki;
        CHOREO_THETA_CONTROLLER_KD = tuning.choreo_theta_kd;
        CHOREO_LOOK_AHEAD = tuning.choreo_look_ahead;

        TRACTOR_BEAM_TRANSLATION_ERROR_MARGIN = tuning.tractor_beam_translation_error_margin;
        TRACTOR_BEAM_STATIC_FRICTION_CONSTANT = tuning.tractor_beam_static_friction;
        TRACTOR_BEAM_CONTROLLER_KP = tuning.tractor_beam_kp;
        TRACTOR_BEAM_CONTROLLER_KI = tuning.tractor_beam_ki;
        TRACTOR_BEAM_CONTROLLER_KD = tuning.tractor_beam_kd;
    }

    /**
     * Control tuning for {@link MwSwerveSubsystem}. Starts at the library defaults; set only what
     * differs, e.g. {@code new Tuning().maxTranslationRate(4.5).choreoLookAhead(0.75)}.
     */
    public static class Tuning {
        private double controller_deadband = 0.1;
        private double max_translation_rate = 5.0;
        private double max_translation_accel = 40.0;
        private double max_crawl_rate = 0.5;
        private double max_angular_rate = 10.0;

        private double heading_kp = 10.0;
        private double heading_ki = 0.0;
        private double heading_kd = 1.0;

        private double stationary_translation_velocity = 0.1;
        private double stationary_angular_velocity = 0.2;

        private double choreo_translation_error_margin = Units.inchesToMeters(1.0);
        private double choreo_velocity_error_margin = 0.2;
        private double choreo_translation_kp = 7.0;
        private double choreo_translation_ki = 0.0;
        private double choreo_translation_kd = 0.0;
        private double choreo_theta_kp = 12.0;
        private double choreo_theta_ki = 0.0;
        private double choreo_theta_kd = 1.0;
        private double choreo_look_ahead = 1.0;

        private double tractor_beam_translation_error_margin = Units.inchesToMeters(0.5);
        private double tractor_beam_static_friction = 0.1;
        private double tractor_beam_kp = 0.0;
        private double tractor_beam_ki = 0.0;
        private double tractor_beam_kd = 0.0;

        /** Deadband applied to raw joystick inputs (default 0.1). */
        public Tuning controllerDeadband(double deadband) {
            controller_deadband = deadband;
            return this;
        }

        /** Max teleop translation speed in m/s (default 5.0). */
        public Tuning maxTranslationRate(double meters_per_second) {
            max_translation_rate = meters_per_second;
            return this;
        }

        /** Max teleop translation acceleration in m/s^2 for the slew limiters (default 40). */
        public Tuning maxTranslationAccel(double meters_per_second_squared) {
            max_translation_accel = meters_per_second_squared;
            return this;
        }

        /** Max speed during crawl (POV) driving in m/s (default 0.5). */
        public Tuning maxCrawlRate(double meters_per_second) {
            max_crawl_rate = meters_per_second;
            return this;
        }

        /** Max angular rate in rad/s (default 10). */
        public Tuning maxAngularRate(double radians_per_second) {
            max_angular_rate = radians_per_second;
            return this;
        }

        /** Rotation-lock heading controller gains (default 10, 0, 1). */
        public Tuning headingGains(double kp, double ki, double kd) {
            heading_kp = kp;
            heading_ki = ki;
            heading_kd = kd;
            return this;
        }

        /** Max translation (m/s) and angular (rad/s) velocity to count as stationary. */
        public Tuning stationaryThresholds(double translation_mps, double angular_radps) {
            stationary_translation_velocity = translation_mps;
            stationary_angular_velocity = angular_radps;
            return this;
        }

        /** Choreo endpoint tolerances: translation (m) and velocity (m/s). */
        public Tuning choreoErrorMargins(double translation_m, double velocity_mps) {
            choreo_translation_error_margin = translation_m;
            choreo_velocity_error_margin = velocity_mps;
            return this;
        }

        /** Choreo x/y feedback gains (default 7, 0, 0). */
        public Tuning choreoTranslationGains(double kp, double ki, double kd) {
            choreo_translation_kp = kp;
            choreo_translation_ki = ki;
            choreo_translation_kd = kd;
            return this;
        }

        /** Choreo heading feedback gains (default 12, 0, 1). */
        public Tuning choreoThetaGains(double kp, double ki, double kd) {
            choreo_theta_kp = kp;
            choreo_theta_ki = ki;
            choreo_theta_kd = kd;
            return this;
        }

        /** Distance (m) the robot may lag the sample before the Choreo timer pauses (default 1). */
        public Tuning choreoLookAhead(double meters) {
            choreo_look_ahead = meters;
            return this;
        }

        /** Tractor beam setpoint tolerance in meters (default 0.5 in). */
        public Tuning tractorBeamErrorMargin(double meters) {
            tractor_beam_translation_error_margin = meters;
            return this;
        }

        /** Fraction of max translation rate added to overcome static friction (default 0.1). */
        public Tuning tractorBeamStaticFriction(double fraction) {
            tractor_beam_static_friction = fraction;
            return this;
        }

        /** Tractor beam distance controller gains (default 0, 0, 0). */
        public Tuning tractorBeamGains(double kp, double ki, double kd) {
            tractor_beam_kp = kp;
            tractor_beam_ki = ki;
            tractor_beam_kd = kd;
            return this;
        }
    }
}
