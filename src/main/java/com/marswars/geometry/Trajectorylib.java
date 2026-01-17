package com.marswars.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Trajectorylib {

    public class TrajectorySol {
        public double velocity;
        public double exit_angle;
        public double heading_angle;
    }

    private double height_;
    private Translation3d target_;
    private InterpolatingDoubleTreeMap range_to_velocity_;
    private static final double g = 9.81;

    public Trajectorylib(Translation3d target, double height, String up_low_Solution) {
        target_ = target;
        height_ = height;

        range_to_velocity_ = new InterpolatingDoubleTreeMap();

    }

    public void addVelocityPoint(double range, double velocity) {
        range_to_velocity_.put(range, velocity);
    }

    private double getVelocity(Pose2d position) {
        double range = target_.toTranslation2d().getDistance(position.getTranslation());
        double velocity = range_to_velocity_.get(range);
        return velocity;
    }

    private double angleTrajectory(Pose2d position, double v) { // unfinished
        double x = target_.toTranslation2d().getDistance(position.getTranslation());

        double numerator = v * v + Math.sqrt(v * v * v * v - g * (g * x * x + 2 * height_ * v * v));
        double denomenator = g * x;

        return Math.atan2(numerator, denomenator);
    }

    private double getHeadingAngle(Pose2d position) {
        double deltaX = position.getX() - target_.getX();
        double deltaY = position.getY() - target_.getY();

        double heading = Math.atan2(deltaY, deltaX); // solves for heading

        return heading;

    }

    public void setTarget(Translation3d target) {
        target_ = target;
    }

    public TrajectorySol getSolution(Pose2d position) {
        TrajectorySol solution = new TrajectorySol();

        solution.velocity = getVelocity(position);
        solution.exit_angle = angleTrajectory(position, solution.velocity);
        solution.heading_angle = getHeadingAngle(position);

        return solution;
    }
}
