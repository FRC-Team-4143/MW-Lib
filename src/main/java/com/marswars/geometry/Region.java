package com.marswars.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** This class models a region of the field. Credit to frc-3061 for base code */
public abstract class Region {
    /**
     * Log the points of the region. These can be visualized using AdvantageScope to confirm that
     * the regions are properly defined.
     */
    public abstract void logPoints();

    /** Returns true if the region contains a given Pose2d. */
    public abstract boolean contains(Pose2d other);

    /** Returns the name of the region for logging purposes. */
    public abstract String getName();

    /** Constructs the region and prepares it for logging. */
    public abstract void constructRegion();

    /**
     * Returns true if the region will contain a given Pose2d after moving at a given speed for a
     * given time.
     * @param robotPose current robot pose
     * @param robotSpeed current robot speed
     * @param time time to project forward
     * @return if the pose is inside the region after moving
     */
    public boolean willContain(Pose2d robotPose, ChassisSpeeds robotSpeed, double time){
        return contains(robotPose.exp(robotSpeed.toTwist2d(time)));
    }
}
