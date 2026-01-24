package com.marswars.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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

    public LaunchTrajectory(Translation3d target, double launch_height, boolean highArc) {
        target_ = target;
        launch_height_ = launch_height;
        height_ = target_.getZ() - launch_height;
        range_to_velocity_ = new InterpolatingDoubleTreeMap();
        highArc_ = highArc;
        useFixedAngle_ = false;
        fixedAngle_ = 0.0;
    }

    public LaunchTrajectory(Translation3d target, double launch_height, double fixedAngle) {
        target_ = target;
        height_ = target_.getZ() - launch_height;
        range_to_velocity_ = new InterpolatingDoubleTreeMap();
        useFixedAngle_ = true;
        fixedAngle_ = fixedAngle;
        highArc_ = false;
    }

    public void addVelocityPoint(double range, double velocity) {
        range_to_velocity_.put(range, velocity);
    }

    public boolean isFixedAngle() {
        return useFixedAngle_;
    }

    public double getFixedAngle() {
        return fixedAngle_;
    }

    private double calculateLaunchVelocity(Pose2d position) {
        double range = target_.toTranslation2d().getDistance(position.getTranslation());
        Double velocity = range_to_velocity_.get(range);
        return velocity != null ? velocity : Double.NaN;
    }

    private double calculateVelocityForAngle(Pose2d position, double angle) {
        double x = target_.toTranslation2d().getDistance(position.getTranslation());
        double h = height_;

        double sin2theta = Math.sin(2 * angle);
        double cos2theta = Math.cos(angle) * Math.cos(angle);

        double denominator = x * sin2theta - 2 * h * cos2theta;
        if (denominator <= 0) return Double.NaN;

        double v_squared = (g * x * x) / denominator;
        if (v_squared <= 0) return Double.NaN;

        return Math.sqrt(v_squared);
    }

    private double calculateLaunchAngle(Pose2d position, double v) {
        double x = target_.toTranslation2d().getDistance(position.getTranslation());

        double discriminant = v * v * v * v - g * (g * x * x + 2 * height_ * v * v);
        if (discriminant < 0) return Double.NaN;

        double numerator = highArc_ ? v * v + Math.sqrt(discriminant) : v * v - Math.sqrt(discriminant);
        double denominator = g * x;

        return Math.atan2(numerator, denominator);
    }

    private double calculateHeadingAngle(Pose2d position, Translation2d aimTarget) {
        double deltaX = aimTarget.getX() - position.getX();
        double deltaY = aimTarget.getY() - position.getY();
        return Math.atan2(deltaY, deltaX);
    }

    public void setTarget(Translation3d target) {
        target_ = target;
        height_ = target.getZ() - launch_height_;
    }

    public TrajectorySol getSolution(Pose2d position) {
        return getSolutionWithVelocity(position, new ChassisSpeeds(0, 0, 0));
    }

    /**
     * Auto-aim while moving.
     * @param position current robot pose
     * @param fieldSpeeds robot field-relative velocity (m/s)
     * @return TrajectorySol containing velocity, exit angle, and heading angle
     */
    public TrajectorySol getSolutionWithVelocity(Pose2d position, ChassisSpeeds fieldSpeeds) {
        TrajectorySol solution = new TrajectorySol();

        // Step 1: compute initial solution assuming stationary
        Translation2d stationaryTarget = target_.toTranslation2d();
        double velocity = useFixedAngle_ ? calculateVelocityForAngle(position, fixedAngle_) : calculateLaunchVelocity(position);

        if (!Double.isFinite(velocity) || velocity <= 0) {
            solution.valid = false;
            solution.velocity = Double.NaN;
            solution.exit_angle = Double.NaN;
            solution.heading_angle = Double.NaN;
            return solution;
        }

        double exitAngle = useFixedAngle_ ? fixedAngle_ : calculateLaunchAngle(position, velocity);

        // Step 2: estimate time of flight
        double horizontalDistance = stationaryTarget.getDistance(position.getTranslation());
        double timeOfFlight = horizontalDistance / (velocity * Math.cos(exitAngle));

        // Step 3: predict lead point
        double leadX = stationaryTarget.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight;
        double leadY = stationaryTarget.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight;
        Translation2d leadTarget = new Translation2d(leadX, leadY);

        // Step 4: recompute range and heading using lead target
        double leadRange = leadTarget.getDistance(position.getTranslation());
        double leadVelocity = useFixedAngle_ ? calculateVelocityForAngle(position, fixedAngle_) : range_to_velocity_.get(leadRange);
        if (!Double.isFinite(leadVelocity) || leadVelocity <= 0) {
            solution.valid = false;
            solution.velocity = Double.NaN;
            solution.exit_angle = Double.NaN;
            solution.heading_angle = Double.NaN;
            return solution;
        }

        double leadExitAngle = useFixedAngle_ ? fixedAngle_ : calculateLaunchAngle(position, leadVelocity);
        double heading = calculateHeadingAngle(position, leadTarget);

        solution.velocity = leadVelocity;
        solution.exit_angle = leadExitAngle;
        solution.heading_angle = heading;
        solution.valid = true;

        return solution;
    }
}
