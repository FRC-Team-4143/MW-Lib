package com.marswars.proxy_server;

import com.marswars.proxy_server.proto.ChassisProxy;
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

    /**
     * Converts a received piece detections message. The first detection fills the top-level fields
     * and every detection (the first included) is listed in {@code allDetections}; a frame with no
     * detections has a {@code detectionCount} of 0.
     *
     * @param msg the piece detections payload of a received envelope
     * @return PieceDetectionData object with parsed information
     */
    public static PieceDetectionData fromProto(ChassisProxy.PieceDetections msg) {
        Timestamp timestamp = Timestamp.fromProto(msg.getStamp());
        String cameraSerial = msg.getCameraSerial();
        int count = msg.getDetections().length();

        ArrayList<PieceDetectionData> all = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            ChassisProxy.Detection detection = msg.getDetections().get(i);
            all.add(new PieceDetectionData(detection.getClassId(), count, i,
                    detection.getThetaXRad(), detection.getThetaYRad(), timestamp, cameraSerial));
        }

        if (count == 0) {
            return new PieceDetectionData(0, 0, 0, 0.0, 0.0, timestamp, cameraSerial);
        }
        PieceDetectionData first = all.get(0);
        return new PieceDetectionData(first.classId, count, 0, first.thetaX, first.thetaY,
                all, timestamp, cameraSerial);
    }
}
