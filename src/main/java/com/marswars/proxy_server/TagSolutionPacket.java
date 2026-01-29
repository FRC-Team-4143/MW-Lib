package com.marswars.proxy_server;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.nio.ByteBuffer;
import java.util.ArrayList;

/**
 * Packet handler for AprilTag detection and pose estimation data.
 * Provides functionality to parse incoming tag solution packets containing
 * robot pose estimates based on detected AprilTags.
 */
public abstract class TagSolutionPacket implements Packet {

    /**
     * Data structure containing AprilTag-based pose solution information.
     */
    public static class TagSolutionData {
        /** Robot pose estimate derived from detected AprilTags */
        public final Pose2d pose;
        /** List of detected AprilTag IDs used for pose estimation */
        public final ArrayList<Integer> detectedIds;
        /** Timestamp when the solution was computed */
        public final Timestamp timestamp;

        public TagSolutionData(Pose2d pose, ArrayList<Integer> detectedIds, Timestamp timestamp) {
            this.pose = pose;
            this.detectedIds = new ArrayList<>(detectedIds);
            this.timestamp = timestamp;
        }
    }

    // Packet type identifier
    public static final int TYPE_ID = 15;
    
    // Byte indices for packet data fields
    private static final int X_POS_IDX = 9;
    private static final int Y_POS_IDX = 13;
    private static final int OMEGA_POS_IDX = 17;
    private static final int DETECTED_TAG_COUNT_IDX = 21;
    private static final int DETECTED_TAG_START_IDX = 25;

    private static final double POSITION_RESOLUTION = 1000.0;



    /**
     * Parses tag solution packet data from a received byte buffer.
     * Extracts timestamp, pose, and detected tag IDs from the buffer.
     * 
     * @param buffer the byte buffer containing the packet data
     * @return TagSolutionData object with parsed information
     */
    public static TagSolutionData updateData(byte[] buffer) {
        // Parse timestamp
        Timestamp timestamp = new Timestamp(
                ByteBuffer.wrap(buffer, TIME_SEC_IDX, 4).getInt(),
                ByteBuffer.wrap(buffer, TIME_NSEC_IDX, 4).getInt());
        
        // Parse position data
        Pose2d pose = new Pose2d(
                ByteBuffer.wrap(buffer, X_POS_IDX, 4).getInt() / POSITION_RESOLUTION,
                ByteBuffer.wrap(buffer, Y_POS_IDX, 4).getInt() / POSITION_RESOLUTION,
                new Rotation2d(
                        ByteBuffer.wrap(buffer, OMEGA_POS_IDX, 4).getInt() / POSITION_RESOLUTION));

        // Parse detected tag IDs
        ArrayList<Integer> detectedIds = new ArrayList<>();
        int detectedTagCount = ByteBuffer.wrap(buffer, DETECTED_TAG_COUNT_IDX, 4).getInt();
        for (int i = 0; i < detectedTagCount; i++) {
            int tagId = ByteBuffer.wrap(buffer, DETECTED_TAG_START_IDX + (i * 4), 4).getInt();
            detectedIds.add(tagId);
        }
        
        return new TagSolutionData(pose, detectedIds, timestamp);
    }
}
