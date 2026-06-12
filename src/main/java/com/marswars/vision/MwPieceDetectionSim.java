package com.marswars.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.marswars.proxy_server.Packet.Timestamp;
import com.marswars.proxy_server.PieceDetectionPacket.PieceDetectionData;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Simulates game piece detections for use with the proxy server in simulation mode.
 * Publishes a Field2d widget ("PieceSim Field") with draggable game piece objects.
 * Pieces dragged in Glass are read back each update, and any piece within a simulated
 * camera's horizontal field of view and range is reported as a detection with a
 * robot-relative pose, matching the format of real piece detection packets.
 *
 * <h3>Usage Example:</h3>
 * <pre>{@code
 * // Create the piece detection simulation (publishes "PieceSim Field" to Glass)
 * MwPieceDetectionSim pieceSim = new MwPieceDetectionSim();
 *
 * // Add a detection camera (same Transform3d convention as MwVisionSim)
 * Transform3d robotToCamera = new Transform3d(
 *     new Translation3d(0.3, 0, 0.3),
 *     new Rotation3d(0, Math.toRadians(-20), 0)
 * );
 * pieceSim.addCamera("piece-camera", robotToCamera);
 *
 * // Add draggable game pieces (classId, starting pose on the field)
 * pieceSim.addGamePiece(0, new Pose2d(4.0, 4.0, Rotation2d.kZero));
 * pieceSim.addGamePiece(0, new Pose2d(6.0, 2.0, Rotation2d.kZero));
 *
 * // In periodic, update with robot pose then collect detections
 * pieceSim.update(robotPose);
 * List<PieceDetectionData> detections = pieceSim.getPieceDetections(robotPose);
 * }</pre>
 */
public class MwPieceDetectionSim {
    private static final String FIELD_LABEL = "PieceSim Field";
    private static final String PIECE_OBJECT_PREFIX = "pieces-class";

    /** Horizontal FOV matching MwVisionSim's default camera (1280x960, 90 degree diagonal) */
    public static final double DEFAULT_HFOV_DEGREES = 77.3;
    /** Maximum detection range matching MwVisionSim's default max sight range */
    public static final double DEFAULT_MAX_RANGE_METERS = 9.0;

    /**
     * Represents a single simulated piece detection camera.
     * Only the geometric properties needed for the 2D FOV check are modeled.
     */
    public static class PieceCameraSim {
        /** Camera name, reported as the camera serial in detection packets */
        public final String cameraName;
        /** Transform from robot center to camera */
        public final Transform3d robotToCamera;
        /** Horizontal field of view (radians) */
        public final double horizontalFovRad;
        /** Maximum detection range (meters) */
        public final double maxRangeMeters;

        public PieceCameraSim(String camera_name, Transform3d robot_to_camera,
                              double horizontal_fov_rad, double max_range_meters) {
            this.cameraName = camera_name;
            this.robotToCamera = robot_to_camera;
            this.horizontalFovRad = horizontal_fov_rad;
            this.maxRangeMeters = max_range_meters;
        }
    }

    private final Field2d field_ = new Field2d();
    private final List<PieceCameraSim> cameras_ = new ArrayList<>();
    private final Map<Integer, List<Pose2d>> pieces_by_class_ = new LinkedHashMap<>();

    /**
     * Creates a new piece detection simulation and publishes its Field2d widget
     * to SmartDashboard as "PieceSim Field".
     */
    public MwPieceDetectionSim() {
        SmartDashboard.putData(FIELD_LABEL, field_);
    }

    /**
     * Adds a simulated piece detection camera with default FOV and range.
     *
     * @param camera_name the name of the camera (used as the camera serial in packets)
     * @param robot_to_camera transform from robot center to camera
     * @return the created PieceCameraSim object
     */
    public PieceCameraSim addCamera(String camera_name, Transform3d robot_to_camera) {
        return addCamera(camera_name, robot_to_camera, DEFAULT_HFOV_DEGREES, DEFAULT_MAX_RANGE_METERS);
    }

    /**
     * Adds a simulated piece detection camera with custom FOV and range.
     *
     * @param camera_name the name of the camera (used as the camera serial in packets)
     * @param robot_to_camera transform from robot center to camera
     * @param hfov_degrees horizontal field of view in degrees
     * @param max_range_meters maximum detection range in meters
     * @return the created PieceCameraSim object
     */
    public PieceCameraSim addCamera(String camera_name, Transform3d robot_to_camera,
                                    double hfov_degrees, double max_range_meters) {
        PieceCameraSim cam = new PieceCameraSim(
                camera_name, robot_to_camera, Math.toRadians(hfov_degrees), max_range_meters);
        cameras_.add(cam);
        return cam;
    }

    /**
     * Adds a draggable game piece to the simulation field.
     *
     * @param class_id classification ID of the piece (groups pieces into one field object per class)
     * @param initial_pose starting pose of the piece on the field
     */
    public void addGamePiece(int class_id, Pose2d initial_pose) {
        List<Pose2d> poses = pieces_by_class_.computeIfAbsent(class_id, k -> new ArrayList<>());
        poses.add(initial_pose);
        field_.getObject(PIECE_OBJECT_PREFIX + class_id).setPoses(poses);
    }

    /**
     * Updates the simulation with the current robot pose.
     * Reads back any piece positions dragged in Glass, then mirrors the robot pose
     * onto the piece sim field. Should be called periodically (every robot loop).
     *
     * @param robot_pose the current simulated robot pose on the field
     */
    public void update(Pose2d robot_pose) {
        if (!RobotBase.isSimulation()) {
            return;
        }

        // Read dragged piece positions back from Glass before anything else.
        // Glass moves poses but never adds/removes them, so sizes stay consistent.
        for (Map.Entry<Integer, List<Pose2d>> entry : pieces_by_class_.entrySet()) {
            List<Pose2d> dragged = field_.getObject(PIECE_OBJECT_PREFIX + entry.getKey()).getPoses();
            if (dragged.size() == entry.getValue().size()) {
                entry.setValue(dragged);
            }
        }

        field_.setRobotPose(robot_pose);
    }

    /**
     * Computes piece detections for all cameras based on the current piece positions.
     * A piece is detected if it lies within a camera's horizontal FOV cone and max range.
     * For each camera seeing N pieces, N detections are produced with detectionCount = N
     * and 1-based detectionIndex, mirroring the real one-detection-per-packet protocol.
     *
     * @param robot_pose the current simulated robot pose on the field
     * @return list of PieceDetectionData with robot-relative piece poses
     */
    public List<PieceDetectionData> getPieceDetections(Pose2d robot_pose) {
        if (!RobotBase.isSimulation()) {
            return new ArrayList<>();
        }

        List<PieceDetectionData> detections = new ArrayList<>();

        // Shared timestamp for this detection cycle
        long timestamp_micros = (long) (Timer.getFPGATimestamp() * 1_000_000);
        Timestamp timestamp = new Timestamp(
                (int) (timestamp_micros / 1_000_000),
                (int) ((timestamp_micros % 1_000_000) * 1000));

        for (PieceCameraSim camera : cameras_) {
            // Project the camera mount to 2D (yaw only, pitch ignored)
            Transform2d robot_to_camera_2d = new Transform2d(
                    camera.robotToCamera.getTranslation().toTranslation2d(),
                    camera.robotToCamera.getRotation().toRotation2d());
            Pose2d camera_pose = robot_pose.transformBy(robot_to_camera_2d);

            // Collect visible pieces for this camera with their class IDs
            List<Integer> visible_class_ids = new ArrayList<>();
            List<Pose2d> visible_robot_to_piece = new ArrayList<>();

            for (Map.Entry<Integer, List<Pose2d>> entry : pieces_by_class_.entrySet()) {
                for (Pose2d piece_pose : entry.getValue()) {
                    Pose2d rel_to_camera = piece_pose.relativeTo(camera_pose);
                    double range = rel_to_camera.getTranslation().getNorm();
                    double bearing = Math.atan2(rel_to_camera.getY(), rel_to_camera.getX());

                    if (range <= 1e-6 || range > camera.maxRangeMeters
                            || Math.abs(bearing) > camera.horizontalFovRad / 2.0) {
                        continue;
                    }

                    // Packet pose is robot-relative; rotation is the bearing to the piece
                    Translation2d rel_translation = piece_pose.relativeTo(robot_pose).getTranslation();
                    Rotation2d rel_bearing = rel_translation.getNorm() > 1e-6
                            ? rel_translation.getAngle()
                            : Rotation2d.kZero;
                    visible_class_ids.add(entry.getKey());
                    visible_robot_to_piece.add(new Pose2d(rel_translation, rel_bearing));
                }
            }

            int detection_count = visible_robot_to_piece.size();
            for (int i = 0; i < detection_count; i++) {
                detections.add(new PieceDetectionData(
                        visible_class_ids.get(i),
                        detection_count,
                        i + 1,
                        visible_robot_to_piece.get(i),
                        timestamp,
                        camera.cameraName));
            }
        }

        return detections;
    }

    /**
     * Gets all cameras in the simulation.
     *
     * @return list of all PieceCameraSim objects
     */
    public List<PieceCameraSim> getCameras() {
        return cameras_;
    }

    /**
     * Gets the Field2d widget used to display and drag game pieces.
     *
     * @return the piece sim Field2d
     */
    public Field2d getField() {
        return field_;
    }
}
