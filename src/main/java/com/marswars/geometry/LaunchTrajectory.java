package com.marswars.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class LaunchTrajectory {

    public class TrajectorySol {
        public double velocity;
        public double exit_angle;
        public double heading_angle;
        public boolean valid;
    }

    private double height_;
    private double launch_height_;
    private boolean highArc_;
    private Translation3d target_;
    private InterpolatingDoubleTreeMap range_to_velocity_;
    private boolean useFixedAngle_;
    private double fixedAngle_;
    private static final double g = 9.81;
    /**
     * Constructor for LaunchTrajectory with variable angle
     * @param target Where you want to shoot (X, Y and Z) in the form of a Translation3d
     * @param launch_height The height of the launcher in meters
     * @param highArc Whether you want the higher arc or the lower arc (true for higher)
     */
    public LaunchTrajectory(Translation3d target, double launch_height, boolean highArc) {
        target_ = target;
        launch_height_ = launch_height;
        height_ = target_.getZ() - launch_height; 
        range_to_velocity_ = new InterpolatingDoubleTreeMap();
        highArc_ = highArc;
        useFixedAngle_ = false;
        fixedAngle_ = 0.0;
    }

    /**
     * Constructor for LaunchTrajectory with fixed angle shooter
     * @param target Where you want to shoot (X, Y and Z) in the form of a Translation3d
     * @param launch_height The height of the launcher in meters
     * @param fixedAngle The fixed exit angle of the shooter in radians
     */
    public LaunchTrajectory(Translation3d target, double launch_height, double fixedAngle) {
        target_ = target;
        height_ = target_.getZ() - launch_height; 
        range_to_velocity_ = new InterpolatingDoubleTreeMap();
        useFixedAngle_ = true;
        fixedAngle_ = fixedAngle;
        highArc_ = false; // Not used in fixed angle mode
    }

    /**
     * Adds points to the linear interpolation table
     * @param range distance from target in meters
     * @param velocity exit velocity for given distance in meters / second
     */
    public void addVelocityPoint(double range, double velocity) {
        range_to_velocity_.put(range, velocity);
    }

    /**
     * Checks if this trajectory uses a fixed angle shooter
     * @return true if using fixed angle, false if using variable angle
     */
    public boolean isFixedAngle() {
        return useFixedAngle_;
    }

    /**
     * Gets the fixed angle (only valid if isFixedAngle() returns true)
     * @return the fixed angle in radians
     */
    public double getFixedAngle() {
        return fixedAngle_;
    }

    /**
     * Calculates the launch velocity needed to hit the target from the given position
     * @param position robot position (X and Y) in the form of a Pose2d
     * @return launch velocity in meters / second, NaN if impossible
     */
    private double calculateLaunchVelocity(Pose2d position) {
        double range = target_.toTranslation2d().getDistance(position.getTranslation());
        Double velocity = range_to_velocity_.get(range);
        return velocity != null ? velocity : Double.NaN;
    }

    /**
     * Calculates the launch velocity needed to hit the target from the given position with the given angle
     * @param position robot position (X and Y) in the form of a Pose2d
     * @param angle launch angle in radians
     * @return launch velocity in meters / second, NaN if impossible
     */
    private double calculateVelocityForAngle(Pose2d position, double angle) {
        double x = target_.toTranslation2d().getDistance(position.getTranslation());
        double h = height_;

        double sin2theta = Math.sin(2 * angle);
        double cos2theta = Math.cos(angle) * Math.cos(angle);

        double denominator = x * sin2theta - 2 * h * cos2theta;
        if (denominator <= 0) {
            return Double.NaN; // invalid trajectory for this angle
        }

        double v_squared = (g * x * x) / denominator;
        if (v_squared <= 0) {
            return Double.NaN;
        }
        return Math.sqrt(v_squared);
    }

    /**
     * Calculates the launch angle needed to hit the target from the given position with the given velocity
     * @param position robot position (X and Y) in the form of a Pose2d
     * @param v launch velocity in meters / second
     * @return launch angle in radians, NaN if impossible
     */
    private double calculateLaunchAngle(Pose2d position, double v) {
        double x = target_.toTranslation2d().getDistance(position.getTranslation());
        
        // Check if trajectory is possible with the given velocity
        double discriminant = v * v * v * v - g * (g * x * x + 2 * height_ * v * v);
        if (discriminant < 0) {
            return Double.NaN; // Impossible trajectory
        }
        
        double numerator;
        if(highArc_ == true){
            numerator = v * v + Math.sqrt(discriminant);
        }
        else{
            numerator = v * v - Math.sqrt(discriminant);
        }
        double denomenator = g * x;

        return Math.atan2(numerator, denomenator);
    }

    /**
     * Calculates the heading angle needed to face the target from the given position
     * @param position robot position (X and Y) in the form of a Pose2d
     * @return heading angle in radians
     */
    private double calculateHeadingAngle(Pose2d position) {
        double deltaX = target_.getX() - position.getX();
        double deltaY = target_.getY() - position.getY();

        double heading = Math.atan2(deltaY, deltaX); // solves for heading

        return heading;

    }
    /**
     * Setter for target; change the target live
     * @param target Z, Y, and Z of target for shot in the form of a Translation3d
     */
    public void setTarget(Translation3d target) {
        target_ = target;
        height_ = target.getZ() - launch_height_;
    }

    /**
     * Setter for highArc; change the arc type live (only valid for variable angle shooters)
     * @param highArc true for higher arc, false for lower arc
     */
    public void setHighArc(boolean highArc) {
        highArc_ = highArc;
    }

    /**
     * Gives a valid shooter trajectory for a given robot position
     * @param position robot position (X and Y) in the form of a Pose2d
     * @return returns a TrajectorySol that contains velocity, exit angle, and heading angle
     */
    public TrajectorySol getSolution(Pose2d position) {
        TrajectorySol solution = new TrajectorySol();
        
        if (useFixedAngle_) {
            // Fixed angle mode: calculate required velocity for the fixed angle
            solution.exit_angle = fixedAngle_;
            solution.velocity = calculateVelocityForAngle(position, fixedAngle_);
            // Valid if velocity is finite and positive
            solution.valid = Double.isFinite(solution.velocity) && solution.velocity > 0;
        } else {
            // Variable angle mode: use velocity lookup table and calculate angle
            solution.velocity = calculateLaunchVelocity(position);
            
            // Check if velocity lookup was successful
            if (!Double.isFinite(solution.velocity) || solution.velocity <= 0) {
                solution.valid = false;
                solution.velocity = Double.NaN;
                solution.exit_angle = Double.NaN;
            } else {
                solution.exit_angle = calculateLaunchAngle(position, solution.velocity);
                // Valid if angle calculation is finite
                solution.valid = Double.isFinite(solution.exit_angle);
            }
        }
        
        solution.heading_angle = calculateHeadingAngle(position);
        
        return solution;
    }

    /**
     * Generates a trajectory visualization as an array of Translation3d points
     * @param solution TrajectorySol obtained from getSolution()
     * @param position robot position (X and Y) in the form of a Pose2d
     * @param maxPoints maximum number of points to calculate (recommended: 20-50 for good resolution)
     * @return array of Translation3d points representing the trajectory, empty array if invalid
     */
    public Translation3d[] getTrajectoryVisualization(TrajectorySol solution, Pose2d position, int maxPoints) {
        if (!solution.valid || maxPoints <= 0) {
            return new Translation3d[0]; // Return empty array for invalid trajectory
        }

        // Calculate trajectory parameters
        double cosHeading = Math.cos(solution.heading_angle);
        double sinHeading = Math.sin(solution.heading_angle);
        
        // Use original launch velocity without robot velocity adjustment
        double launchAngle = useFixedAngle_ ? fixedAngle_ : 
            calculateLaunchAngle(position, solution.velocity);
        
        if (!Double.isFinite(launchAngle)) {
            return new Translation3d[0];
        }

        // Calculate time of flight
        double vx = solution.velocity * Math.cos(launchAngle);
        double vy = solution.velocity * Math.sin(launchAngle);
        
        // Time when projectile hits target height: t = (vy + sqrt(vy^2 + 2*g*height_)) / g
        double discriminant = vy * vy + 2 * g * height_;
        if (discriminant < 0) {
            return new Translation3d[0];
        }
        
        double timeOfFlight = (vy + Math.sqrt(discriminant)) / g;
        if (timeOfFlight <= 0) {
            return new Translation3d[0];
        }

        // Calculate trajectory points efficiently
        Translation3d[] trajectory = new Translation3d[maxPoints];
        double timeStep = timeOfFlight / (maxPoints - 1);
        
        for (int i = 0; i < maxPoints; i++) {
            double t = i * timeStep;
            
            // Calculate position at time t
            double horizontalDistance = vx * t;
            double verticalPosition = launch_height_ + vy * t - 0.5 * g * t * t;
            
            // Convert to world coordinates
            double x = position.getX() + horizontalDistance * cosHeading;
            double y = position.getY() + horizontalDistance * sinHeading;
            
            trajectory[i] = new Translation3d(x, y, verticalPosition);
        }
        
        return trajectory;
    }

    /**
     * Generates a trajectory visualization with default parameters
     * @param solution TrajectorySol obtained from getSolution()
     * @param position robot position (X and Y) in the form of a Pose2d
     * @return array of Translation3d points representing the trajectory (30 points default)
     */
    public Translation3d[] getTrajectoryVisualization(TrajectorySol solution, Pose2d position) {
        return getTrajectoryVisualization(solution, position, 30);
    }
}
