package com.marswars.proxy_server;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.marswars.proxy_server.proto.ChassisProxy;
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

    /**
     * Converts a received tag solution message.
     *
     * @param msg the tag solution payload of a received envelope
     * @return TagSolutionData object with parsed information
     */
    public static TagSolutionData fromProto(ChassisProxy.TagSolution msg) {
        ChassisProxy.Pose2d pose = msg.getRobotPose();
        ArrayList<Integer> detectedIds = new ArrayList<>(msg.getTagIds().length());
        for (int i = 0; i < msg.getTagIds().length(); i++) {
            detectedIds.add(msg.getTagIds().get(i));
        }
        return new TagSolutionData(
                new Pose2d(pose.getXM(), pose.getYM(), new Rotation2d(pose.getThetaRad())),
                detectedIds,
                Timestamp.fromProto(msg.getStamp()),
                msg.getCameraSerial());
    }
}
