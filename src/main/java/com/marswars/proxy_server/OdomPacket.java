package com.marswars.proxy_server;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import java.nio.ByteBuffer;

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

    // Packet type identifier
    public static final int TYPE_ID = 30;
    
    // Byte indices for odometry data fields
    private static final int X_POSITION_IDX = 9;
    private static final int Y_POSITION_IDX = 13;
    private static final int THETA_POSITION_IDX = 17;
    private static final int X_VARIANCE_IDX = 21;
    private static final int Y_VARIANCE_IDX = 25;
    private static final int THETA_VARIANCE_IDX = 29;
    private static final int X_VELOCITY_IDX = 33;
    private static final int Y_VELOCITY_IDX = 37;
    private static final int THETA_VELOCITY_IDX = 41;

    private static final double ODOMETRY_RESOLUTION = 1000.0;

    /**
     * Parses odometry packet data from a received byte buffer.
     * Extracts timestamp, pose, velocity, and variance information from the buffer.
     * 
     * @param buffer the byte buffer containing the packet data
     * @return OdometryData object with parsed information
     */
    public static OdometryData updateData(byte[] buffer) {
        // Parse timestamp
        Timestamp timestamp = new Timestamp(
                ByteBuffer.wrap(buffer, TIME_SEC_IDX, 4).getInt(),
                ByteBuffer.wrap(buffer, TIME_NSEC_IDX, 4).getInt());
        
        // Parse position data
        Pose2d pose = new Pose2d(
                ByteBuffer.wrap(buffer, X_POSITION_IDX, 4).getInt() / ODOMETRY_RESOLUTION,
                ByteBuffer.wrap(buffer, Y_POSITION_IDX, 4).getInt() / ODOMETRY_RESOLUTION,
                new Rotation2d(
                        ByteBuffer.wrap(buffer, THETA_POSITION_IDX, 4).getInt() / ODOMETRY_RESOLUTION));

        // Parse velocity data
        Twist2d twist = new Twist2d(
                ByteBuffer.wrap(buffer, X_VELOCITY_IDX, 4).getInt() / ODOMETRY_RESOLUTION,
                ByteBuffer.wrap(buffer, Y_VELOCITY_IDX, 4).getInt() / ODOMETRY_RESOLUTION,
                ByteBuffer.wrap(buffer, THETA_VELOCITY_IDX, 4).getInt() / ODOMETRY_RESOLUTION);

        // Parse position variances for uncertainty estimation
        double[] variances = new double[3];
        variances[0] = ByteBuffer.wrap(buffer, X_VARIANCE_IDX, 4).getInt() / ODOMETRY_RESOLUTION;
        variances[1] = ByteBuffer.wrap(buffer, Y_VARIANCE_IDX, 4).getInt() / ODOMETRY_RESOLUTION;
        variances[2] = ByteBuffer.wrap(buffer, THETA_VARIANCE_IDX, 4).getInt() / ODOMETRY_RESOLUTION;
        
        return new OdometryData(pose, twist, variances, timestamp);
    }
}
