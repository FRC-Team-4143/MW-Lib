package com.marswars.proxy_server;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
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
        /** Camera serial number / identifier */
        public final String cameraSerial;

        public TagSolutionData(Pose2d pose, ArrayList<Integer> detectedIds, Timestamp timestamp, String cameraSerial) {
            this.pose = pose;
            this.detectedIds = new ArrayList<>(detectedIds);
            this.timestamp = timestamp;
            this.cameraSerial = cameraSerial;
        }
    }

    // Packet type identifier
    public static final int TYPE_ID = 15;
    
    // Byte indices for packet data fields
    private static final int DOUBLE_SIZE = 8; // Size of double values in bytes
    private static final int INT_SIZE = 4; // Size of integer values in bytes
    private static final int SHORT_SIZE = 2; // Size of short values in bytes

    // Camera serial string starts after the header
    private static final int CAMERA_SERIAL_LEN_IDX = HEADER_SIZE;

    private static final double POSITION_RESOLUTION = 1E6; // Resolution for position data (e.g., 1 unit = 1 millionth of a meter)



    /**
     * Parses tag solution packet data from a received byte buffer.
     * Extracts timestamp, camera serial, pose, and detected tag IDs from the buffer.
     * 
     * @param buffer the byte buffer containing the packet data
     * @return TagSolutionData object with parsed information
     */
    public static TagSolutionData updateData(byte[] buffer) {
        // Parse timestamp
        Timestamp timestamp = new Timestamp(
                ByteBuffer.wrap(buffer, TIME_SEC_IDX, 4).getInt(),
                ByteBuffer.wrap(buffer, TIME_NSEC_IDX, 4).getInt());
        
        // Parse camera serial string
        int cameraSerialLen = ByteBuffer.wrap(buffer, CAMERA_SERIAL_LEN_IDX, SHORT_SIZE).getShort() & 0xFFFF;
        int cameraSerialStartIdx = CAMERA_SERIAL_LEN_IDX + SHORT_SIZE;
        String cameraSerial = new String(buffer, cameraSerialStartIdx, cameraSerialLen);
        
        // Calculate indices for remaining fields (after variable-length string)
        int xPosIdx = cameraSerialStartIdx + cameraSerialLen;
        int yPosIdx = xPosIdx + DOUBLE_SIZE;
        int omegaPosIdx = yPosIdx + DOUBLE_SIZE;
        int detectedTagCountIdx = omegaPosIdx + DOUBLE_SIZE;
        int detectedTagStartIdx = detectedTagCountIdx + INT_SIZE;
        
        // Parse position data
        Pose2d pose = new Pose2d(
                ByteBuffer.wrap(buffer, xPosIdx, 8).getLong() / POSITION_RESOLUTION,
                ByteBuffer.wrap(buffer, yPosIdx, 8).getLong() / POSITION_RESOLUTION,
                new Rotation2d(
                        ByteBuffer.wrap(buffer, omegaPosIdx, 8).getLong() / POSITION_RESOLUTION));

        // Parse detected tag IDs
        ArrayList<Integer> detectedIds = new ArrayList<>();
        int detectedTagCount = ByteBuffer.wrap(buffer, detectedTagCountIdx, 4).getInt();
        for (int i = 0; i < detectedTagCount; i++) {
            int tagId = ByteBuffer.wrap(buffer, detectedTagStartIdx + (i), 1).get();
            detectedIds.add(tagId);
        }
        
        return new TagSolutionData(pose, detectedIds, timestamp, cameraSerial);
    }
}
