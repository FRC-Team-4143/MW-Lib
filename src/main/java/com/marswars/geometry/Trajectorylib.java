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
    private boolean highArc_;
    private Translation3d target_;
    private InterpolatingDoubleTreeMap range_to_velocity_;
    private static final double g = 9.81;
    /**
     * Constructor for Trajectorylib
     * @param target Where you want to shoot (X, Y and Z) in the form of a Translation3d
     * @param highArc Whether you want the higher arc or the lower arc (true for higher)
     */
    public Trajectorylib(Translation3d target, boolean highArc) {
        target_ = target;
        height_ = target_.getZ();

        range_to_velocity_ = new InterpolatingDoubleTreeMap();
        highArc_ = highArc;
    }

    /**
     * Adds points to the linear interpolation table
     * @param range distance from target in meters
     * @param velocity exit velocity for given distance in meters / second
     */
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
        
        double numerator;
        if(highArc_ == true){
            numerator = v * v + Math.sqrt(v * v * v * v - g * (g * x * x + 2 * height_ * v * v));
        }
        else{
            numerator = v * v - Math.sqrt(v * v * v * v - g * (g * x * x + 2 * height_ * v * v));
        }
        double denomenator = g * x;

        return Math.atan2(numerator, denomenator);
    }

    private double getHeadingAngle(Pose2d position) {
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
    }
    /**
     * Gives a valid shooter trajectory for a given robot position
     * @param position robot position (X and Y) in the form of a Pose2d
     * @return returns a TrajectorySol that contains velocity, exit angle, and heading angle
     */
    public TrajectorySol getSolution(Pose2d position) {
        TrajectorySol solution = new TrajectorySol();

        solution.velocity = getVelocity(position);
        solution.exit_angle = angleTrajectory(position, solution.velocity);
        solution.heading_angle = getHeadingAngle(position);

        return solution;
    }
}
