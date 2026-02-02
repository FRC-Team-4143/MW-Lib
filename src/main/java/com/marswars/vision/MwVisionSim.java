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
    
    private final VisionSystemSim visionSim;
    private final List<CameraSimulation> cameras;
    private final AprilTagFieldLayout fieldLayout;
    
    /**
     * Represents a single simulated camera with its properties and transform.
     */
    public static class CameraSimulation {
        public final PhotonCamera camera;
        public final PhotonCameraSim cameraSim;
        public final Transform3d robotToCamera;
        
        public CameraSimulation(String cameraName, SimCameraProperties properties, Transform3d robotToCamera) {
            this.camera = new PhotonCamera(cameraName);
            this.cameraSim = new PhotonCameraSim(camera, properties);
            this.robotToCamera = robotToCamera;
        }
    }
    
    /**
     * Creates a new VisionSimulation with the specified field layout.
     * The vision system will be labeled "proxy-vision-sim" in NetworkTables.
     * 
     * @param fieldLayout the AprilTag field layout to use for the simulation
     */
    public MwVisionSim(AprilTagFieldLayout fieldLayout) {
        this.visionSim = new VisionSystemSim(VISION_SIM_LABEL);
        this.cameras = new ArrayList<>();
        this.fieldLayout = fieldLayout;
        
        if (fieldLayout != null) {
            visionSim.addAprilTags(fieldLayout);
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
        this.visionSim = new VisionSystemSim(VISION_SIM_LABEL);
        this.cameras = new ArrayList<>();
        
        AprilTagFieldLayout layout = null;
        try {
            layout = AprilTagFieldLayout.loadField(field);
            visionSim.addAprilTags(layout);
        } catch (Exception e) {
            System.err.println("Failed to load AprilTag field layout: " + e.getMessage());
            e.printStackTrace();
        }
        this.fieldLayout = layout;
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
        double cameraHeight = 0.3; // meters up from robot center
        double cameraOffset = 0.3; // meters from robot center
        double cameraAngle = Math.toRadians(-20); // angle down from horizontal
        
        // Front camera - facing forward
        Transform3d frontCamera = new Transform3d(
            new Translation3d(cameraOffset, 0, cameraHeight),
            new Rotation3d(0, cameraAngle, 0)
        );
        addCamera("camera-front", frontCamera);
        
        // Left camera - facing left (90 degrees)
        Transform3d leftCamera = new Transform3d(
            new Translation3d(0, cameraOffset, cameraHeight),
            new Rotation3d(0, cameraAngle, Math.toRadians(90))
        );
        addCamera("camera-left", leftCamera);
        
        // Right camera - facing right (-90 degrees)
        Transform3d rightCamera = new Transform3d(
            new Translation3d(0, -cameraOffset, cameraHeight),
            new Rotation3d(0, cameraAngle, Math.toRadians(-90))
        );
        addCamera("camera-right", rightCamera);
        
        // Back camera - facing backward (180 degrees)
        Transform3d backCamera = new Transform3d(
            new Translation3d(-cameraOffset, 0, cameraHeight),
            new Rotation3d(0, cameraAngle, Math.toRadians(180))
        );
        addCamera("camera-back", backCamera);
        
        System.out.println("VisionSimulation: Added 4 default cameras (front, left, right, back)");
    }
    
    /**
     * Adds a simulated camera to the vision system.
     * 
     * @param cameraName the name of the camera
     * @param robotToCamera transform from robot center to camera
     * @return the created CameraSimulation object
     */
    public CameraSimulation addCamera(String cameraName, Transform3d robotToCamera) {
        return addCamera(cameraName, createDefaultCameraProperties(), robotToCamera);
    }
    
    /**
     * Adds a simulated camera with custom properties to the vision system.
     * 
     * @param cameraName the name of the camera
     * @param properties the camera properties (resolution, FOV, noise, etc.)
     * @param robotToCamera transform from robot center to camera
     * @return the created CameraSimulation object
     */
    public CameraSimulation addCamera(String cameraName, SimCameraProperties properties, Transform3d robotToCamera) {
        CameraSimulation camSim = new CameraSimulation(cameraName, properties, robotToCamera);
        cameras.add(camSim);
        visionSim.addCamera(camSim.cameraSim, robotToCamera);
        
        // Enable camera streams for debugging
        camSim.cameraSim.enableRawStream(true);
        camSim.cameraSim.enableProcessedStream(true);
        
        return camSim;
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
     * @param robotPose the current simulated robot pose on the field
     */
    public void update(Pose2d robotPose) {
        if (!RobotBase.isSimulation()) {
            return;
        }
        
        // Convert Pose2d to Pose3d (robot is on the ground, z=0)
        Pose3d robotPose3d = new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            0.0,
            new Rotation3d(0, 0, robotPose.getRotation().getRadians())
        );
        
        visionSim.update(robotPose3d);
    }
    
    /**
     * Gets all cameras in the simulation.
     * 
     * @return list of all CameraSimulation objects
     */
    public List<CameraSimulation> getCameras() {
        return cameras;
    }
    
    /**
     * Gets the vision system simulator.
     * 
     * @return the VisionSystemSim instance
     */
    public VisionSystemSim getVisionSim() {
        return visionSim;
    }
    
    /**
     * Gets the AprilTag field layout being used.
     * 
     * @return the AprilTagFieldLayout
     */
    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }
    
    /**
     * Extracts vision data from all cameras and converts to TagSolutionData format.
     * This method processes the latest results from all simulated cameras and generates
     * tag solution data compatible with the proxy server format.
     * 
     * @param robotPose the current robot pose (used for timestamp synchronization)
     * @return list of TagSolutionData from all cameras with valid targets
     */
    public List<TagSolutionPacket.TagSolutionData> getTagSolutions(Pose2d robotPose) {
        if (!RobotBase.isSimulation()) {
            return new ArrayList<>();
        }
        
        List<TagSolutionPacket.TagSolutionData> solutions = new ArrayList<>();
        
        for (CameraSimulation camSim : cameras) {
            var result = camSim.camera.getLatestResult();
            
            // Skip if no targets detected
            if (!result.hasTargets()) {
                continue;
            }
            
            // Get the best target (or could iterate through all targets)
            var bestTarget = result.getBestTarget();
            
            // Collect all detected AprilTag IDs
            ArrayList<Integer> detectedIds = new ArrayList<>();
            for (var target : result.getTargets()) {
                if (target.getFiducialId() >= 0) {
                    detectedIds.add(target.getFiducialId());
                }
            }
            
            // Skip if no valid fiducial IDs
            if (detectedIds.isEmpty()) {
                continue;
            }
            
            // Try to get the pose estimate from the camera
            Pose2d estimatedPose = robotPose; // Default to current robot pose
            
            // If we have a transform from the target, we could estimate pose
            // For now, we'll use the robot's current pose as the "estimated" pose
            // In a real scenario, you'd use the camera's pose estimation
            if (bestTarget.getBestCameraToTarget() != null) {
                // Get the AprilTag pose from the field layout
                try {
                    Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(bestTarget.getFiducialId());
                    if (tagPoseOpt.isPresent()) {
                        // Calculate robot pose from tag detection
                        Pose3d tagPose3d = tagPoseOpt.get();
                        Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
                        Transform3d robotToCamera = camSim.robotToCamera;
                        
                        // Robot pose = Tag pose - (Robot to Camera + Camera to Target)
                        Pose3d cameraPose = tagPose3d.transformBy(cameraToTarget.inverse());
                        Pose3d estimatedRobotPose = cameraPose.transformBy(robotToCamera.inverse());
                        
                        estimatedPose = estimatedRobotPose.toPose2d();
                    }
                } catch (Exception e) {
                    // If pose estimation fails, use current robot pose
                    System.err.println("Failed to estimate pose from tag: " + e.getMessage());
                }
            }
            
            // Create timestamp (using system time for simulation)
            long timestampMicros = (long)(result.getTimestampSeconds() * 1_000_000);
            Timestamp timestamp = new Timestamp(
                (int)(timestampMicros / 1_000_000),
                (int)((timestampMicros % 1_000_000) * 1000)
            );
            
            // Create TagSolutionData
            TagSolutionPacket.TagSolutionData solution = new TagSolutionPacket.TagSolutionData(
                estimatedPose,
                detectedIds,
                timestamp
            );
            
            solutions.add(solution);
        }
        
        return solutions;
    }
    
    /**
     * Adjusts a camera's transform relative to the robot.
     * Useful for simulating moving mechanisms like turrets.
     * 
     * @param cameraSim the camera simulation to adjust
     * @param newRobotToCamera the new transform from robot to camera
     */
    public void adjustCamera(CameraSimulation cameraSim, Transform3d newRobotToCamera) {
        visionSim.adjustCamera(cameraSim.cameraSim, newRobotToCamera);
    }
}
