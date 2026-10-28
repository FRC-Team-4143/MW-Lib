package com.marswars.proxy_server;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import com.marswars.proxy_server.proto.ChassisProxy;

/**
 * Packet handler for robot odometry data.
 * Provides functionality to parse incoming packets containing
 * robot pose, velocity, and position variance information.
 */
public abstract class OdomPacket implements Packet {

    /**
     * Data structure containing robot odometry information.
     */
    public static class OdometryData {
        /** Robot pose (position and rotation) */
        public final Pose2d pose;
        /** Robot velocity (linear and angular) */
        public final Twist2d twist;
        /** Position variances [x, y, theta] for uncertainty estimation */
        public final double[] variances;
        /** Timestamp when the odometry was recorded */
        public final Timestamp timestamp;

        public OdometryData(Pose2d pose, Twist2d twist, double[] variances, Timestamp timestamp) {
            this.pose = pose;
            this.twist = twist;
            this.variances = variances.clone();
            this.timestamp = timestamp;
        }
    }

    /**
     * Converts a received odometry message.
     *
     * @param msg the odometry payload of a received envelope
     * @return OdometryData object with parsed information
     */
    public static OdometryData fromProto(ChassisProxy.Odometry msg) {
        ChassisProxy.Pose2d pose = msg.getPose();
        return new OdometryData(
                new Pose2d(pose.getXM(), pose.getYM(), new Rotation2d(pose.getThetaRad())),
                new Twist2d(msg.getVxMps(), msg.getVyMps(), msg.getOmegaRadps()),
                new double[] {msg.getXVariance(), msg.getYVariance(), msg.getThetaVariance()},
                Timestamp.fromProto(msg.getStamp()));
    }
}
