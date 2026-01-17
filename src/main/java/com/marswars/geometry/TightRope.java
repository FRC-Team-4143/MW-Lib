package com.marswars.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

public class TightRope {
    public Pose2d pose_a_;
    public Pose2d pose_b_;
    private StructArrayPublisher<Translation2d> array_publisher_;
    private String name_;

    /**
     * Create a TightRope between two poses
     *
     * @param pose_a Pose2d of one end of the tightrope
     * @param pose_b Pose2d of the other end of the tightrope
     * @param name Name of the tightrope for logging
     */
    public TightRope(Pose2d pose_a, Pose2d pose_b, String name) {
        pose_a_ = pose_a;
        pose_b_ = pose_b;
        name_ = name;

        array_publisher_ =
                NetworkTableInstance.getDefault()
                        .getStructArrayTopic("TightRopes/" + name, Translation2d.struct)
                        .publish();

        logPoints();
    }

    /**
     * Log the two endpoints of the tightrope. These can be visualized using AdvantageScope to
     * confirm that the tightropes are properly defined.
     */
    public void logPoints() {
        Translation2d[] points_ = new Translation2d[2];
        points_[0] = pose_a_.getTranslation();
        points_[1] = pose_b_.getTranslation();
        array_publisher_.set(points_);
    }

    /**
     * Get the name of the tightrope
     *
     * @return the name of the tightrope
     */
    public String getName_() {
        return name_;
    }
}
