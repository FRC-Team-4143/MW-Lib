package com.marswars.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import com.marswars.proxy_server.Packet.Timestamp;
import com.marswars.proxy_server.TagSolutionPacket;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Handles PhotonVision simulation for generating vision data in simulation mode.
 * Creates a simulated vision system with cameras and AprilTag field layout,
 * and generates vision target data that can be integrated with the proxy server.
 * 
 * <p>Tag solutions mimic a real coprocessor: when a camera sees two or more tags in a picture the
 * solution is the camera's multi-tag PnP pose and its detected ids are the tags used in that solve;
 * when only one tag is visible the pose comes from that single tag and the detected ids hold only
 * that tag. At most one solution is produced per camera per picture.
 * 
 * <p>The vision system label is hard-coded to "proxy-vision-sim" for NetworkTables.
 * The field layout must be specified at construction time and cannot be changed.
 * 
 * <h3>Usage Example:</h3>
 * <pre>{@code
 * // Load the field layout
 * AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
 *     AprilTagFields.k2025Reefscape
 * );
 * 
 * // Create vision simulation with the field layout
 * VisionSimulation visionSim = new VisionSimulation(fieldLayout);
 * 
 * // Or use the convenience constructor
 * VisionSimulation visionSim = new VisionSimulation(AprilTagFields.k2025Reefscape);
 * 
 * // Add default cameras (front, left, right, back)
 * visionSim.addDefaultCameras();
 * 
 * // Or add custom cameras
 * Transform3d robotToCamera = new Transform3d(
 *     new Translation3d(0.5, 0, 0.3),
 *     new Rotation3d(0, Math.toRadians(-15), 0)
 * );
 * visionSim.addCamera("frontCamera", robotToCamera);
 * 
 * // In periodic, update with robot pose
 * visionSim.update(robotPose);
 * 
 * // Get vision solutions
 * List<TagSolutionData> solutions = visionSim.getTagSolutions(robotPose);
 * }</pre>
 */
public class MwVisionSim {
    private static final String VISION_SIM_LABEL = "proxy-vision-sim";
    
    private final VisionSystemSim vision_sim_;
    private final List<CameraSimulation> cameras_;
    private final AprilTagFieldLayout field_layout_;
    
    /**
     * Represents a single simulated camera with its properties and transform.
     */
    public static class CameraSimulation {
        public final PhotonCamera camera;
        public final PhotonCameraSim cameraSim;
        public final Transform3d robotToCamera;
        
        /** Timestamp of the last picture a tag solution was emitted for (FPGA seconds). */
        private double last_emitted_timestamp_ = Double.NEGATIVE_INFINITY;
        
        /** 
         * Creates a new CameraSimulation with the specified parameters.
         * 
         * @param camera_name the name of the camera
         * @param properties the camera properties (resolution, FOV, noise, etc.)
         * @param robot_to_camera transform from robot center to camera
         */
        public CameraSimulation(String camera_name, SimCameraProperties properties, Transform3d robot_to_camera) {
            this.camera = new PhotonCamera(camera_name);
            this.cameraSim = new PhotonCameraSim(camera, properties);
            this.robotToCamera = robot_to_camera;
        }

        /** 
         * Creates a new CameraSimulation with the specified parameters and max sight range.
         * 
         * @param camera_name the name of the camera
         * @param properties the camera properties (resolution, FOV, noise, etc.)
         * @param robot_to_camera transform from robot center to camera
         * @param max_sight_range maximum sight range of the camera in meters
         */
        public CameraSimulation(String camera_name, SimCameraProperties properties, Transform3d robot_to_camera, double max_sight_range) {
            this.camera = new PhotonCamera(camera_name);
            this.cameraSim = new PhotonCameraSim(camera, properties);
            this.robotToCamera = robot_to_camera;
            this.cameraSim.setMaxSightRange(max_sight_range);
        }

        /** 
         * Creates a new CameraSimulation whose multi-tag solve uses the given field layout.
         * (The constructors without a layout solve against PhotonLib's default field.)
         * 
         * @param camera_name the name of the camera
         * @param properties the camera properties (resolution, FOV, noise, etc.)
         * @param robot_to_camera transform from robot center to camera
         * @param max_sight_range maximum sight range of the camera in meters
         * @param field_layout the AprilTag field layout used for the multi-tag solve
         */
        public CameraSimulation(String camera_name, SimCameraProperties properties, Transform3d robot_to_camera, double max_sight_range, AprilTagFieldLayout field_layout) {
            this.camera = new PhotonCamera(camera_name);
            this.cameraSim = new PhotonCameraSim(camera, properties, field_layout);
            this.robotToCamera = robot_to_camera;
            this.cameraSim.setMaxSightRange(max_sight_range);
        }
    }
    
    /**
     * Creates a new VisionSimulation with the specified field layout.
     * The vision system will be labeled "proxy-vision-sim" in NetworkTables.
     * 
     * @param field_layout the AprilTag field layout to use for the simulation
     */
    public MwVisionSim(AprilTagFieldLayout field_layout) {
        this.vision_sim_ = new VisionSystemSim(VISION_SIM_LABEL);
        this.cameras_ = new ArrayList<>();
        this.field_layout_ = field_layout;
        
        if (field_layout != null) {
            vision_sim_.addAprilTags(field_layout);
        }
    }
    
    /**
     * Creates a new VisionSimulation with a predefined field layout.
     * This is a convenience constructor that loads the field layout automatically.
     * The vision system will be labeled "proxy-vision-sim" in NetworkTables.
     * 
     * @param field the predefined AprilTag field (e.g., AprilTagFields.k2025Reefscape)
     */
    public MwVisionSim(AprilTagFields field) {
        this.vision_sim_ = new VisionSystemSim(VISION_SIM_LABEL);
        this.cameras_ = new ArrayList<>();
        
        AprilTagFieldLayout layout = null;
        try {
            layout = AprilTagFieldLayout.loadField(field);
            vision_sim_.addAprilTags(layout);
        } catch (Exception e) {
            System.err.println("Failed to load AprilTag field layout: " + e.getMessage());
            e.printStackTrace();
        }
        this.field_layout_ = layout;
    }
    
    /**
     * Adds a default set of four cameras positioned around the robot.
     * Creates front, left, right, and back cameras at standard positions:
     * - Front: 0.3m forward, 0.3m up, angled 20° down
     * - Left: 0.3m left, 0.3m up, angled 20° down, facing 90° left
     * - Right: 0.3m right, 0.3m up, angled 20° down, facing 90° right
     * - Back: 0.3m back, 0.3m up, angled 20° down, facing 180° back
     * 
     * All cameras use default camera properties.
     */
    public void addDefaultCameras() {
        // Camera mounting height and distance from center
        double camera_height = 0.3; // meters up from robot center
        double camera_offset = 0.3; // meters from robot center
        double camera_angle = Math.toRadians(-20); // angle down from horizontal
        
        // Front camera - facing forward
        Transform3d front_camera = new Transform3d(
            new Translation3d(camera_offset, 0, camera_height),
            new Rotation3d(0, camera_angle, 0)
        );
        addCamera("camera-front", front_camera);
        
        // Left camera - facing left (90 degrees)
        Transform3d left_camera = new Transform3d(
            new Translation3d(0, camera_offset, camera_height),
            new Rotation3d(0, camera_angle, Math.toRadians(90))
        );
        addCamera("camera-left", left_camera);
        
        // Right camera - facing right (-90 degrees)
        Transform3d right_camera = new Transform3d(
            new Translation3d(0, -camera_offset, camera_height),
            new Rotation3d(0, camera_angle, Math.toRadians(-90))
        );
        addCamera("camera-right", right_camera);
        
        // Back camera - facing backward (180 degrees)
        Transform3d back_camera = new Transform3d(
            new Translation3d(-camera_offset, 0, camera_height),
            new Rotation3d(0, camera_angle, Math.toRadians(180))
        );
        addCamera("camera-back", back_camera);
        
        System.out.println("VisionSimulation: Added 4 default cameras (front, left, right, back)");
    }
    
    /**
     * Adds a simulated camera to the vision system.
     * 
     * @param camera_name the name of the camera
     * @param robot_to_camera transform from robot center to camera
     * @return the created CameraSimulation object
     */
    public CameraSimulation addCamera(String camera_name, Transform3d robot_to_camera) {
        // Use default camera properties and max sight range of 9.0 meters (roughly half the field length)
        return addCamera(camera_name, createDefaultCameraProperties(), robot_to_camera, 9.0);
    }
    
    /**
     * Adds a simulated camera with custom properties to the vision system.
     * 
     * @param camera_name the name of the camera
     * @param properties the camera properties (resolution, FOV, noise, etc.)
     * @param robot_to_camera transform from robot center to camera
     * @param max_sight_range maximum sight range of the camera in meters
     * @return the created CameraSimulation object
     */
    public CameraSimulation addCamera(String camera_name, SimCameraProperties properties, Transform3d robot_to_camera, double max_sight_range) {
        // Multi-tag solves must use this simulation's field, not PhotonLib's default field
        CameraSimulation cam_sim = field_layout_ != null
            ? new CameraSimulation(camera_name, properties, robot_to_camera, max_sight_range, field_layout_)
            : new CameraSimulation(camera_name, properties, robot_to_camera, max_sight_range);
        cameras_.add(cam_sim);
        vision_sim_.addCamera(cam_sim.cameraSim, robot_to_camera);
        
        // Disable camera streams by default to reduce overhead
        cam_sim.cameraSim.enableRawStream(false);
        cam_sim.cameraSim.enableProcessedStream(false);
        
        return cam_sim;
    }
    
    /**
     * Creates default camera properties typical for FRC vision systems.
     * - 960x720 resolution
     * - 70 degree diagonal FOV
     * - Minimal noise and latency for simulation
     * - 30 FPS
     * 
     * @return SimCameraProperties with default settings
     */
    public static SimCameraProperties createDefaultCameraProperties() {
        SimCameraProperties props = new SimCameraProperties();
        // Set camera resolution and FOV (1280x960, 90 degree diagonal)
        props.setCalibration(1280, 960, Rotation2d.fromDegrees(90));
        // Add some realistic noise
        props.setCalibError(0.2, 0.05);
        // Set framerate
        props.setFPS(30);
        // Add latency (average 30ms with 5ms std dev)
        props.setAvgLatencyMs(30);
        props.setLatencyStdDevMs(5);
        return props;
    }
    
    /**
     * Updates the vision simulation with the current robot pose.
     * This should be called periodically (every robot loop) with the simulated robot pose.
     * 
     * @param robot_pose the current simulated robot pose on the field
     */
    public void update(Pose2d robot_pose) {
        if (!RobotBase.isSimulation()) {
            return;
        }
        
        // Convert Pose2d to Pose3d (robot is on the ground, z=0)
        Pose3d robot_pose_3d = new Pose3d(
            robot_pose.getX(),
            robot_pose.getY(),
            0.0,
            new Rotation3d(0, 0, robot_pose.getRotation().getRadians())
        );
        
        vision_sim_.update(robot_pose_3d);
    }
    
    /**
     * Gets all cameras in the simulation.
     * 
     * @return list of all CameraSimulation objects
     */
    public List<CameraSimulation> getCameras() {
        return cameras_;
    }
    
    /**
     * Gets the vision system simulator.
     * 
     * @return the VisionSystemSim instance
     */
    public VisionSystemSim getVisionSim() {
        return vision_sim_;
    }
    
    /**
     * Gets the AprilTag field layout being used.
     * 
     * @return the AprilTagFieldLayout
     */
    public AprilTagFieldLayout getFieldLayout() {
        return field_layout_;
    }
    
    /**
     * Extracts vision data from all cameras and converts to TagSolutionData format.
     * This method processes the latest results from all simulated cameras and generates
     * tag solution data compatible with the proxy server format.
     * 
     * <p>Each solution behaves like one from a real coprocessor:
     * <ul>
     * <li>If the camera sees two or more tags, the pose is the camera's multi-tag PnP solve
     *     (accurate to a few cm) and the detected ids are the tags used in that solve.</li>
     * <li>If only one tag is visible, the pose is estimated from that tag alone (noisy, can be
     *     meters off) and the detected ids hold just that tag, so a "two or more tags" filter
     *     rejects it.</li>
     * <li>At most one solution is returned per camera per new picture: if a camera's latest
     *     result has not advanced since the last call, that camera is skipped. The timestamp is
     *     the picture's timestamp on the FPGA clock.</li>
     * </ul>
     * 
     * @param robot_pose the robot pose, used only as the fallback pose if a single-tag estimate is unavailable
     * @return list of TagSolutionData from all cameras with a new picture containing valid targets
     */
    public List<TagSolutionPacket.TagSolutionData> getTagSolutions(Pose2d robot_pose) {
        if (!RobotBase.isSimulation()) {
            return new ArrayList<>();
        }
        
        List<TagSolutionPacket.TagSolutionData> solutions = new ArrayList<>();
        
        for (CameraSimulation cam_sim : cameras_) {
            var result = cam_sim.camera.getLatestResult();
            
            // Skip pictures already emitted (the robot loop runs faster than the camera frame rate)
            if (result.getTimestampSeconds() <= cam_sim.last_emitted_timestamp_) {
                continue;
            }
            cam_sim.last_emitted_timestamp_ = result.getTimestampSeconds();
            
            // Skip if no targets detected
            if (!result.hasTargets()) {
                continue;
            }
            
            // Tag ids used in the solution, and the robot pose estimated from them
            ArrayList<Integer> detected_ids = new ArrayList<>();
            Pose2d estimated_pose = robot_pose; // Default to current robot pose
            
            var multi_tag = result.getMultiTagResult();
            if (multi_tag.isPresent()) {
                // Two or more tags: use the camera's multi-tag PnP solve (field to camera)
                for (short id : multi_tag.get().fiducialIDsUsed) {
                    detected_ids.add((int) id);
                }
                Pose3d camera_pose = new Pose3d().transformBy(multi_tag.get().estimatedPose.best);
                estimated_pose = camera_pose.transformBy(cam_sim.robotToCamera.inverse()).toPose2d();
            } else {
                // Single tag: estimate the robot pose from the best target only
                var best_target = result.getBestTarget();
                if (best_target.getFiducialId() < 0) {
                    continue;
                }
                detected_ids.add(best_target.getFiducialId());
                
                // Robot pose = Tag pose - (Robot to Camera + Camera to Target)
                Optional<Pose3d> tag_pose_opt = field_layout_.getTagPose(best_target.getFiducialId());
                if (tag_pose_opt.isPresent()) {
                    Pose3d camera_pose = tag_pose_opt.get().transformBy(best_target.getBestCameraToTarget().inverse());
                    estimated_pose = camera_pose.transformBy(cam_sim.robotToCamera.inverse()).toPose2d();
                }
            }
            
            // Create timestamp (using system time for simulation)
            long timestamp_micros = (long)(result.getTimestampSeconds() * 1_000_000);
            Timestamp timestamp = new Timestamp(
                (int)(timestamp_micros / 1_000_000),
                (int)((timestamp_micros % 1_000_000) * 1000)
            );
            
            // Get camera name as serial identifier
            String cameraSerial = cam_sim.camera.getName();
            
            // Create TagSolutionData
            TagSolutionPacket.TagSolutionData solution = new TagSolutionPacket.TagSolutionData(
                estimated_pose,
                detected_ids,
                timestamp,
                cameraSerial
            );
            
            solutions.add(solution);
        }
        
        return solutions;
    }
    
    /**
     * Adjusts a camera's transform relative to the robot.
     * Useful for simulating moving mechanisms like turrets.
     * 
     * @param camera_sim the camera simulation to adjust
     * @param new_robot_to_camera the new transform from robot to camera
     */
    public void adjustCamera(CameraSimulation camera_sim, Transform3d new_robot_to_camera) {
        vision_sim_.adjustCamera(camera_sim.cameraSim, new_robot_to_camera);
    }
    
    /**
     * Enables or disables the raw camera stream for all cameras.
     * The raw stream shows the original camera feed without any processing overlay.
     * Disabled by default to reduce overhead.
     * 
     * @param enable true to enable the raw stream, false to disable
     */
    public void enableRawStream(boolean enable) {
        for (CameraSimulation cam_sim : cameras_) {
            cam_sim.cameraSim.enableRawStream(enable);
        }
    }
    
    /**
     * Enables or disables the processed camera stream for all cameras.
     * The processed stream shows the camera feed with vision processing overlays (e.g., detected targets).
     * Disabled by default to reduce overhead.
     * 
     * @param enable true to enable the processed stream, false to disable
     */
    public void enableProcessedStream(boolean enable) {
        for (CameraSimulation cam_sim : cameras_) {
            cam_sim.cameraSim.enableProcessedStream(enable);
        }
    }
}
