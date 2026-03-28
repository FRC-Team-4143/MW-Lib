package com.marswars.proxy_server;

import java.nio.ByteBuffer;
import java.util.ArrayList;

/**
 * Packet handler for game piece detection data.
 * Provides functionality to parse incoming packets containing
 * information about detected game pieces and their locations.
 */
public abstract class PieceDetectionPacket implements Packet {

    /**
     * Data structure containing game piece detection information.
     */
    public static class PieceDetectionData {
        /** Classification ID of the detected piece */
        public final int classId;
        /** Number of detections in this packet */
        public final int detectionCount;
        /** Index of this detection in the sequence */
        public final int detectionIndex;
        /** Horizontal angle to the detected piece (radians) */
        public final double thetaX;
        /** Vertical angle to the detected piece (radians) */
        public final double thetaY;
        /** List of all piece detections in this packet */
        public final ArrayList<PieceDetectionData> allDetections;
        /** Timestamp when the detection was made */
        public final Timestamp timestamp;
        /** Camera serial number / identifier */
        public final String cameraSerial;

        public PieceDetectionData(int classId, int detectionCount, int detectionIndex,
                                double thetaX, double thetaY, 
                                ArrayList<PieceDetectionData> allDetections, Timestamp timestamp, String cameraSerial) {
            this.classId = classId;
            this.detectionCount = detectionCount;
            this.detectionIndex = detectionIndex;
            this.thetaX = thetaX;
            this.thetaY = thetaY;
            this.allDetections = new ArrayList<>(allDetections);
            this.timestamp = timestamp;
            this.cameraSerial = cameraSerial;
        }
        
        // Simple constructor for individual detections
        public PieceDetectionData(int classId, int detectionCount, int detectionIndex,
                                double thetaX, double thetaY, Timestamp timestamp, String cameraSerial) {
            this(classId, detectionCount, detectionIndex, thetaX, thetaY, new ArrayList<>(), timestamp, cameraSerial);
        }
    }

    // Packet type identifier
    public static final int TYPE_ID = 10;
    
    // Byte indices for piece detection data fields
    private static final int SHORT_SIZE = 2; // Size of short values in bytes
    private static final int CAMERA_SERIAL_LEN_IDX = HEADER_SIZE;

    private static final double ANGLE_RESOLUTION = 1e6;

    /**
     * Parses piece detection packet data from a received byte buffer.
     * Extracts timestamp, camera serial, and game piece detection information from the buffer.
     * 
     * @param buffer the byte buffer containing the packet data
     * @return PieceDetectionData object with parsed information
     */
    public static PieceDetectionData updateData(byte[] buffer) {
        // Parse timestamp
        Timestamp timestamp = new Timestamp(
                ByteBuffer.wrap(buffer, TIME_SEC_IDX, 4).getInt(),
                ByteBuffer.wrap(buffer, TIME_NSEC_IDX, 4).getInt());
        
        // Parse camera serial string
        int cameraSerialLen = ByteBuffer.wrap(buffer, CAMERA_SERIAL_LEN_IDX, SHORT_SIZE).getShort() & 0xFFFF;
        int cameraSerialStartIdx = CAMERA_SERIAL_LEN_IDX + SHORT_SIZE;
        String cameraSerial = new String(buffer, cameraSerialStartIdx, cameraSerialLen);
        
        // Calculate indices for remaining fields (after variable-length string)
        int detectionCountIdx = cameraSerialStartIdx + cameraSerialLen;
        int detectionIndexIdx = detectionCountIdx + 4;
        int classIdIdx = detectionIndexIdx + 4;
        int thetaXIdx = classIdIdx + 1;
        int thetaYIdx = thetaXIdx + 8;
        
        // Parse detection metadata
        int detectionCount = ByteBuffer.wrap(buffer, detectionCountIdx, 4).getInt();
        int detectionIndex = ByteBuffer.wrap(buffer, detectionIndexIdx, 4).getInt();
        
        // Handle empty detection case
        if (detectionCount == 0 || detectionIndex == 0) {
            return new PieceDetectionData(0, detectionCount, detectionIndex, 0.0, 0.0, timestamp, cameraSerial);
        }

        // Parse detection data
        int classId = ByteBuffer.wrap(buffer, classIdIdx, 1).get();
        double thetaX = ByteBuffer.wrap(buffer, thetaXIdx, 8).getLong() / ANGLE_RESOLUTION;
        double thetaY = ByteBuffer.wrap(buffer, thetaYIdx, 8).getLong() / ANGLE_RESOLUTION;
        
        return new PieceDetectionData(classId, detectionCount, detectionIndex, 
                                    thetaX, thetaY, timestamp, cameraSerial);
    }
}
