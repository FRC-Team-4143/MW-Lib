package com.marswars.geometry;

import com.marswars.data_structures.TunableDoubleMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * LaunchCalculator - Map-based trajectory calculator for shooting
 * 
 * <p>Uses interpolating tree maps to calculate launch parameters based on distance to target.
 * Supports motion compensation for shooting while moving. Based on code from FRC teams 6328 and 254.
 * 
 * <p>Example usage:
 * <pre>{@code
 * // Create calculator with robot-to-launcher transform
 * LaunchCalculator calculator = new LaunchCalculator(
 *     "LaunchCalculator",
 *     new Transform2d(new Translation2d(-0.3, 0.0), new Rotation2d())
 * );
 * 
 * // Add data points for hood angle (distance in meters -> angle in radians)
 * calculator.addHoodAnglePoint(1.5, Math.toRadians(20.0));
 * calculator.addHoodAnglePoint(2.0, Math.toRadians(25.0));
 * calculator.addHoodAnglePoint(3.0, Math.toRadians(30.0));
 * 
 * // Add data points for flywheel speed (distance in meters -> speed in rad/s)
 * calculator.addFlywheelSpeedPoint(1.5, 200.0);
 * calculator.addFlywheelSpeedPoint(2.0, 220.0);
 * calculator.addFlywheelSpeedPoint(3.0, 250.0);
 * 
 * // Add data points for time of flight (distance in meters -> time in seconds)
 * calculator.addTimeOfFlightPoint(1.5, 0.8);
 * calculator.addTimeOfFlightPoint(2.0, 1.0);
 * calculator.addTimeOfFlightPoint(3.0, 1.2);
 * 
 * // Calculate parameters during operation
 * LaunchParameters params = calculator.calculateLaunchParameters(
 *     robotPose,
 *     robotVelocity,
 *     targetPose
 * );
 * 
 * if (params.isValid()) {
 *     // Use params.driveAngle, params.hoodAngle, params.flywheelSpeed, etc.
 * }
 * }</pre>
 */
public class LaunchCalculator {
    
    /**
     * Container for calculated launch parameters
     * 
     * <p>This class contains all the parameters needed to execute a shot, plus diagnostic information.
     */
    public static class LaunchParameters {
        /** Whether the calculated parameters are valid (within min/max range) */
        public final boolean is_valid;
        
        /** 
         * Target heading angle to face the goal
         * 
         * <p><b>ACTION REQUIRED:</b> Set your heading mechanism's rotation setpoint to this angle.
         * This could be:
         * <ul>
         *   <li>Robot rotation (swerve/tank drive)</li>
         *   <li>Turret rotation (if you have a turret)</li>
         *   <li>Both (robot rotation + turret for fine adjustment)</li>
         * </ul>
         */
        public final Rotation2d heading_angle;
        
        /** 
         * Angular velocity for the heading mechanism (rad/s)
         * 
         * <p><b>ACTION REQUIRED:</b> Use this for feedforward in your heading controller
         * (robot rotation or turret rotation) to improve tracking performance when moving
         */
        public final double heading_velocity;
        
        /** 
         * Hood angle in radians
         * 
         * <p><b>ACTION REQUIRED:</b> Set your hood/pivot mechanism to this angle
         */
        public final double hood_angle;
        
        /** 
         * Hood angular velocity (rad/s) for feedforward
         * 
         * <p><b>ACTION REQUIRED:</b> Use this for feedforward in your hood controller
         * to improve tracking when the shot distance is changing rapidly
         */
        public final double hood_velocity;
        
        /** 
         * Flywheel speed in rad/s
         * 
         * <p><b>ACTION REQUIRED:</b> Set your flywheel/shooter wheels to this speed
         */
        public final double flywheel_speed;
        
        /** 
         * Calculated distance to target with lookahead (meters)
         * 
         * <p><b>DIAGNOSTIC ONLY:</b> This is the distance used for map lookups after
         * accounting for motion compensation. Useful for logging and debugging.
         */
        public final double distance;
        
        /** 
         * Distance to target without motion compensation (meters)
         * 
         * <p><b>DIAGNOSTIC ONLY:</b> This is the direct distance from launcher to target.
         * Useful for comparing against the compensated distance to see how much the
         * motion compensation adjusted the shot.
         */
        public final double distance_no_lookahead;
        
        /** 
         * Estimated time of flight (seconds)
         * 
         * <p><b>DIAGNOSTIC ONLY:</b> How long the projectile will be in the air.
         * Can be useful for visualization or for timing follow-up actions.
         */
        public final double time_of_flight;
        
        public LaunchParameters(
                boolean is_valid,
                Rotation2d heading_angle,
                double heading_velocity,
                double hood_angle,
                double hood_velocity,
                double flywheel_speed,
                double distance,
                double distance_no_lookahead,
                double time_of_flight) {
            this.is_valid = is_valid;
            this.heading_angle = heading_angle;
            this.heading_velocity = heading_velocity;
            this.hood_angle = hood_angle;
            this.hood_velocity = hood_velocity;
            this.flywheel_speed = flywheel_speed;
            this.distance = distance;
            this.distance_no_lookahead = distance_no_lookahead;
            this.time_of_flight = time_of_flight;
        }
        
        /**
         * Creates an invalid parameter set (for when target is out of range)
         */
        public static LaunchParameters invalid() {
            return new LaunchParameters(
                false,
                new Rotation2d(),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            );
        }
    }
    
    // Interpolating maps for lookup tables
    private final TunableDoubleMap hood_angle_map_;
    private final TunableDoubleMap flywheel_speed_map_;
    private final TunableDoubleMap time_of_flight_map_;
    
    // Range limits
    private double min_distance_ = 1.0;
    private double max_distance_ = 6.0;
    
    // Timing parameters
    private double phase_delay_ = 0.03; // seconds - processing and actuator delay
    private double loop_period_ = 0.02; // seconds - default loop period
    
    // Transform from robot center to launcher
    private final Transform2d robot_to_launcher_;
    
    // Filters for velocity calculations
    private final LinearFilter hood_angle_filter_;
    private final LinearFilter heading_angle_filter_;
    
    // State tracking for velocity calculations
    private double last_hood_angle_ = Double.NaN;
    private Rotation2d last_heading_angle_ = null;
    
    // Cache for latest parameters
    private LaunchParameters latest_parameters_ = null;
    
    /**
     * Creates a new LaunchCalculator
     * 
     * @param logging_prefix Prefix for DogLog tunable entries
     * @param robot_to_launcher Transform from robot center to launcher position
     */
    public LaunchCalculator(String logging_prefix, Transform2d robot_to_launcher) {
        this(logging_prefix, robot_to_launcher, 0.02);
    }
    
    /**
     * Creates a new LaunchCalculator with custom loop period
     * 
     * @param logging_prefix Prefix for DogLog tunable entries
     * @param robot_to_launcher Transform from robot center to launcher position
     * @param loop_period Robot loop period in seconds
     */
    public LaunchCalculator(String logging_prefix, Transform2d robot_to_launcher, double loop_period) {
        this.robot_to_launcher_ = robot_to_launcher;
        this.loop_period_ = loop_period;
        
        // Initialize tunable interpolating maps
        this.hood_angle_map_ = new TunableDoubleMap(logging_prefix + "HoodAngle");
        this.flywheel_speed_map_ = new TunableDoubleMap(logging_prefix + "FlywheelSpeed");
        this.time_of_flight_map_ = new TunableDoubleMap(logging_prefix + "TimeOfFlight");
        
        // Initialize filters
        this.hood_angle_filter_ = LinearFilter.movingAverage((int) (0.1 / loop_period));
        this.heading_angle_filter_ = LinearFilter.movingAverage((int) (0.8 / loop_period));
    }
    
    /**
     * Adds a hood angle data point to the interpolation map
     * 
     * @param distance Distance to target in meters
     * @param angle Hood angle in radians
     */
    public void addHoodAnglePoint(double distance, double angle) {
        hood_angle_map_.put(distance, angle);
    }
    
    /**
     * Adds a flywheel speed data point to the interpolation map
     * 
     * @param distance Distance to target in meters
     * @param speed Flywheel speed in rad/s
     */
    public void addFlywheelSpeedPoint(double distance, double speed) {
        flywheel_speed_map_.put(distance, speed);
    }
    
    /**
     * Adds a time of flight data point to the interpolation map
     * 
     * @param distance Distance to target in meters
     * @param time Time of flight in seconds
     */
    public void addTimeOfFlightPoint(double distance, double time) {
        time_of_flight_map_.put(distance, time);
    }
    
    /**
     * Sets the minimum valid shooting distance
     * 
     * @param min_distance Minimum distance in meters
     */
    public void setMinDistance(double min_distance) {
        this.min_distance_ = min_distance;
    }
    
    /**
     * Sets the maximum valid shooting distance
     * 
     * @param max_distance Maximum distance in meters
     */
    public void setMaxDistance(double max_distance) {
        this.max_distance_ = max_distance;
    }
    
    /**
     * Sets the phase delay (processing and actuator lag)
     * 
     * @param phase_delay Phase delay in seconds
     */
    public void setPhaseDelay(double phase_delay) {
        this.phase_delay_ = phase_delay;
    }
    
    /**
     * Gets the minimum time of flight from the map
     * 
     * @return Minimum time of flight in seconds
     */
    public double getMinTimeOfFlight() {
        return time_of_flight_map_.get(min_distance_);
    }
    
    /**
     * Gets the maximum time of flight from the map
     * 
     * @return Maximum time of flight in seconds
     */
    public double getMaxTimeOfFlight() {
        return time_of_flight_map_.get(max_distance_);
    }
    
    /**
     * Gets a naive (non-interpolated) time of flight estimate
     * 
     * @param distance Distance to target in meters
     * @return Time of flight in seconds
     */
    public double getNaiveTimeOfFlight(double distance) {
        return time_of_flight_map_.get(distance);
    }
    
    /**
     * Clears the cached launch parameters, forcing recalculation on next call
     */
    public void clearCache() {
        latest_parameters_ = null;
    }
    
    /**
     * Calculates launch parameters with motion compensation
     * 
     * @param robot_pose Current robot pose on the field
     * @param robot_velocity Current robot velocity (field-relative)
     * @param target_pose Target pose to shoot at
     * @return Calculated launch parameters
     */
    public LaunchParameters calculateLaunchParameters(
            Pose2d robot_pose,
            ChassisSpeeds robot_velocity,
            Pose2d target_pose) {
        return calculateLaunchParameters(
            robot_pose,
            robot_velocity,
            target_pose.getTranslation()
        );
    }
    
    /**
     * Calculates launch parameters with motion compensation
     * 
     * <p>This is the main method for calculating a shot. It handles:
     * <ul>
     *   <li>Phase delay compensation (accounts for processing and mechanism lag)</li>
     *   <li>Motion compensation (predicts where the robot will be when the shot releases)</li>
     *   <li>Launcher offset from robot center (so the launcher points at target, not the robot center)</li>
     *   <li>Velocity feedforward for smooth tracking</li>
     * </ul>
     * 
     * <p><b>How to use the returned parameters:</b>
     * <pre>{@code
     * LaunchParameters params = calculator.calculateLaunchParameters(pose, velocity, target);
     * 
     * if (params.is_valid) {
     *     // ACTION REQUIRED - Apply these 5 parameters to your robot mechanisms:
     *     driveSubsystem.setRotation(params.heading_angle, params.heading_velocity);
     *     hoodSubsystem.setAngle(params.hood_angle, params.hood_velocity);
     *     shooterSubsystem.setSpeed(params.flywheel_speed);
     *     
     *     // DIAGNOSTIC ONLY - Log these 3 parameters for debugging/visualization:
     *     DogLog.log("Shooter/TargetDistance", params.distance);
     *     DogLog.log("Shooter/DirectDistance", params.distance_no_lookahead);
     *     DogLog.log("Shooter/TimeOfFlight", params.time_of_flight);
     * }
     * }</pre>
     * 
     * <p><b>Parameter Summary:</b>
     * <ul>
     *   <li><b>ACTION REQUIRED (5):</b> heading_angle, heading_velocity, hood_angle, hood_velocity, flywheel_speed</li>
     *   <li><b>DIAGNOSTIC ONLY (3):</b> distance, distance_no_lookahead, time_of_flight</li>
     * </ul>
     * 
     * @param robot_pose Current robot pose on the field
     * @param robot_velocity Current robot velocity (field-relative)
     * @param target_translation Target translation to shoot at
     * @return Calculated launch parameters (check is_valid before using!)
     */
    public LaunchParameters calculateLaunchParameters(
            Pose2d robot_pose,
            ChassisSpeeds robot_velocity,
            Translation2d target_translation) {
        
        // ============================================================================
        // STEP 1: Phase Delay Compensation
        // ============================================================================
        // Account for the time between calculating the shot and actually releasing it.
        // This includes processing delay, communication lag, and actuator response time.
        // We predict where the robot will be when the shot actually fires.
        Pose2d estimated_pose = robot_pose.exp(
            new Twist2d(
                robot_velocity.vxMetersPerSecond * phase_delay_,
                robot_velocity.vyMetersPerSecond * phase_delay_,
                robot_velocity.omegaRadiansPerSecond * phase_delay_
            )
        );
        
        // ============================================================================
        // STEP 2: Calculate Launcher Position
        // ============================================================================
        // Transform from the robot center to the actual launcher position.
        // This accounts for the launcher being offset from the robot's center of rotation.
        Pose2d launcher_pose = estimated_pose.transformBy(robot_to_launcher_);
        double launcher_to_target_distance = launcher_pose.getTranslation()
            .getDistance(target_translation);
        
        // ============================================================================
        // STEP 3: Calculate Launcher Velocity
        // ============================================================================
        // Determine how fast the launcher is moving across the field.
        // This is a simplified calculation that assumes the launcher velocity equals
        // the robot's translational velocity (ignoring angular velocity effects on
        // the launcher's position, which is typically a small error).
        double launcher_velocity_x = robot_velocity.vxMetersPerSecond;
        double launcher_velocity_y = robot_velocity.vyMetersPerSecond;
        
        // ============================================================================
        // STEP 4: Iterative Motion Compensation (Lookahead)
        // ============================================================================
        // The projectile takes time to reach the target. During that time, our moving
        // robot will be in a different position. We need to aim at where the robot
        // WILL BE when the projectile arrives, not where it is now.
        //
        // This is solved iteratively:
        // 1. Get initial time of flight estimate based on current distance
        // 2. Calculate where robot will be after that time of flight
        // 3. Recalculate distance and time of flight from that new position
        // 4. Repeat until it converges (typically converges in 3-5 iterations)
        double time_of_flight = time_of_flight_map_.get(launcher_to_target_distance);
        Pose2d lookahead_pose = launcher_pose;
        double lookahead_launcher_to_target_distance = launcher_to_target_distance;
        
        // Iterate to converge on accurate lookahead (up to 20 iterations)
        for (int i = 0; i < 20; i++) {
            // Look up how long the shot will take from this distance
            time_of_flight = time_of_flight_map_.get(lookahead_launcher_to_target_distance);
            
            // Calculate how far the launcher will travel during flight time
            double offset_x = launcher_velocity_x * time_of_flight;
            double offset_y = launcher_velocity_y * time_of_flight;
            
            // Project the launcher's future position
            lookahead_pose = new Pose2d(
                launcher_pose.getTranslation().plus(new Translation2d(offset_x, offset_y)),
                launcher_pose.getRotation()
            );
            
            // Recalculate distance from this future launcher position to target
            lookahead_launcher_to_target_distance = 
                target_translation.getDistance(lookahead_pose.getTranslation());
        }
        
        // ============================================================================
        // STEP 5: Calculate Robot Heading
        // ============================================================================
        // Transform back from the lookahead launcher pose to get the corresponding
        // robot pose, then calculate what heading the robot needs to aim at the target.
        // This accounts for the launcher being offset from the robot's center.
        Pose2d lookahead_robot_pose = lookahead_pose.transformBy(robot_to_launcher_.inverse());
        
        Rotation2d heading_angle = calculateHeadingAngleWithOffset(
            lookahead_robot_pose,
            target_translation
        );
        
        // ============================================================================
        // STEP 6: Look Up Mechanism Setpoints
        // ============================================================================
        // Use the compensated distance to look up the required hood angle and
        // flywheel speed from our interpolating maps. These maps are tuned
        // empirically and can be adjusted live via DogLog.
        double hood_angle = hood_angle_map_.get(lookahead_launcher_to_target_distance);
        double flywheel_speed = flywheel_speed_map_.get(lookahead_launcher_to_target_distance);
        
        // ============================================================================
        // STEP 7: Calculate Velocity Feedforward
        // ============================================================================
        // For smooth tracking, we need velocity feedforward terms. These tell the
        // controllers how fast the setpoints are changing so they can compensate.
        // We use filters to smooth out the velocity calculations and reduce noise.
        
        // Initialize filters on first call
        if (last_heading_angle_ == null) {
            last_heading_angle_ = heading_angle;
        }
        if (Double.isNaN(last_hood_angle_)) {
            last_hood_angle_ = hood_angle;
        }
        
        // Calculate hood angular velocity (derivative of hood angle)
        double hood_velocity = hood_angle_filter_.calculate(
            (hood_angle - last_hood_angle_) / loop_period_
        );
        last_hood_angle_ = hood_angle;
        
        // Calculate heading angular velocity (derivative of heading angle)
        double heading_velocity = heading_angle_filter_.calculate(
            heading_angle.minus(last_heading_angle_).getRadians() / loop_period_
        );
        last_heading_angle_ = heading_angle;
        
        // ============================================================================
        // STEP 8: Validate and Return
        // ============================================================================
        // Check if the shot is within our valid range. If the target is too close
        // or too far, mark the parameters as invalid (but still return them for
        // diagnostic purposes).
        boolean is_valid = lookahead_launcher_to_target_distance >= min_distance_
            && lookahead_launcher_to_target_distance <= max_distance_;
        
        // Package all calculated parameters together
        latest_parameters_ = new LaunchParameters(
            is_valid,
            heading_angle,          // Robot/turret rotation setpoint
            heading_velocity,       // Robot/turret rotation feedforward
            hood_angle,             // Hood/pivot angle setpoint
            hood_velocity,          // Hood/pivot angle feedforward
            flywheel_speed,         // Shooter wheel speed setpoint
            lookahead_launcher_to_target_distance,  // Compensated distance (for logging)
            launcher_to_target_distance,            // Direct distance (for comparison)
            time_of_flight                          // Projectile flight time (for logging)
        );
        
        return latest_parameters_;
    }
    
    /**
     * Gets the most recently calculated launch parameters (cached)
     * 
     * @return Latest launch parameters, or invalid parameters if none calculated yet
     */
    public LaunchParameters getLatestParameters() {
        if (latest_parameters_ == null) {
            return LaunchParameters.invalid();
        }
        return latest_parameters_;
    }
    
    /**
     * Calculates the robot heading angle accounting for launcher offset from robot center
     * 
     * <p>When the launcher is offset from the robot center, the robot needs to be positioned
     * such that the launcher (not the robot center) points at the target.
     * 
     * @param robot_pose Current robot pose
     * @param target Target translation
     * @return Heading angle for the robot
     */
    private Rotation2d calculateHeadingAngleWithOffset(
            Pose2d robot_pose,
            Translation2d target) {
        
        // Calculate angle from robot to target
        Rotation2d field_to_target_angle = target.minus(robot_pose.getTranslation()).getAngle();
        
        // Calculate angle correction needed due to launcher offset
        double target_distance = target.getDistance(robot_pose.getTranslation());
        Rotation2d offset_angle = new Rotation2d(
            Math.asin(
                MathUtil.clamp(
                    robot_to_launcher_.getTranslation().getY() / target_distance,
                    -1.0,
                    1.0
                )
            )
        );
        
        // Combine angles: field to target + offset correction + launcher rotation
        Rotation2d heading_angle = field_to_target_angle
            .plus(offset_angle)
            .plus(robot_to_launcher_.getRotation());
        
        return heading_angle;
    }
    
    /**
     * Calculates the aimed robot pose for stationary shooting
     * 
     * <p>Useful for calculating setpoints for autonomous routines or visualizations.
     * 
     * @param robot_translation Current robot translation
     * @param target Target translation
     * @return Robot pose aimed at target
     */
    public Pose2d getStationaryAimedPose(
            Translation2d robot_translation,
            Translation2d target) {
        Pose2d robot_pose = new Pose2d(robot_translation, new Rotation2d());
        Rotation2d heading_angle = calculateHeadingAngleWithOffset(robot_pose, target);
        return new Pose2d(robot_translation, heading_angle);
    }
    
    /**
     * Builder class for convenient configuration of LaunchCalculator
     */
    public static class Builder {
        private String logging_prefix_ = "LaunchCalculator/";
        private Transform2d robot_to_launcher_ = new Transform2d();
        private double loop_period_ = 0.02;
        private double min_distance_ = 1.0;
        private double max_distance_ = 6.0;
        private double phase_delay_ = 0.03;
        
        /**
         * Sets the logging prefix for DogLog tunables
         */
        public Builder withLoggingPrefix(String logging_prefix) {
            this.logging_prefix_ = logging_prefix;
            return this;
        }
        
        /**
         * Sets the transform from robot center to launcher
         */
        public Builder withRobotToLauncher(Transform2d transform) {
            this.robot_to_launcher_ = transform;
            return this;
        }
        
        /**
         * Sets the robot loop period
         */
        public Builder withLoopPeriod(double loop_period) {
            this.loop_period_ = loop_period;
            return this;
        }
        
        /**
         * Sets the minimum valid distance
         */
        public Builder withMinDistance(double min_distance) {
            this.min_distance_ = min_distance;
            return this;
        }
        
        /**
         * Sets the maximum valid distance
         */
        public Builder withMaxDistance(double max_distance) {
            this.max_distance_ = max_distance;
            return this;
        }
        
        /**
         * Sets the phase delay
         */
        public Builder withPhaseDelay(double phase_delay) {
            this.phase_delay_ = phase_delay;
            return this;
        }
        
        /**
         * Builds the LaunchCalculator
         */
        public LaunchCalculator build() {
            LaunchCalculator calculator = new LaunchCalculator(logging_prefix_, robot_to_launcher_, loop_period_);
            calculator.setMinDistance(min_distance_);
            calculator.setMaxDistance(max_distance_);
            calculator.setPhaseDelay(phase_delay_);
            return calculator;
        }
    }
}
