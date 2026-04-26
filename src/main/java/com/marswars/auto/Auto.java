package com.marswars.auto;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.marswars.geometry.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;

/**
 * Base class for autonomous routines that load Choreo trajectories and expose
 * common path utilities.
 */
public class Auto extends SequentialCommandGroup {
  // Trajectory info for visualization / path following
  protected LinkedHashMap<String, ChoreoTrajectory> trajectories_ = new LinkedHashMap<>();
  private ArrayList<Pose2d[]> trajectory_list_ = new ArrayList<>();

  /** Creates a new autonomous routine container with a default name. */
  public Auto() {
    this.setName(getClass().getSimpleName());
  }

  /**
   * Registers a trajectory to be part of the auto. The actual trajectory will be
   * loaded once the auto is selected, allowing for pre-caching of trajectories
   * without
   * loading them all at once during initialization.
   * 
   * @param name The name of the trajectory to load
   * 
   * @apiNote You can load multiple trajectories by the same name; their poses
   *          will be concatenated for visualization and path following.
   */
  protected void loadTrajectory(String name) {
    trajectories_.put(name, null);
  }

  /**
   * Loads all registered trajectories from Choreo and processes them for use in
   * the auto. This should be called once after the auto is selected to ensure all
   * paths are ready before execution
   * 
   * @param is_red_alliance true if the robot is on the red alliance, false for blue (used for flipping trajectories)
   */
  @SuppressWarnings("unchecked")
  public void cacheTrajetories(boolean is_red_alliance) {
    for (String name : trajectories_.keySet()) {
      // request the choreo trajectory to be loaded

      Trajectory<SwerveSample> traj = (Trajectory<SwerveSample>) choreo.Choreo.loadTrajectory(name).get();

      // load the trajectory with event markers into our typed ChoreoTrajectory class
      // and store it
      ChoreoTrajectory choreoTraj = new ChoreoTrajectory(traj, is_red_alliance);
      trajectories_.put(name, choreoTraj);

      // load the points for visualization
      trajectory_list_.add(traj.getPoses());
    }
  }

  /**
   * Get a loaded trajectory by name.
   *
   * @param name The name of the trajectory
   * @return The typed Choreo trajectory
   */
  protected ChoreoTrajectory getTrajectory(String name) {
    return trajectories_.get(name);
  }

  /**
   * Gets the starting pose for the first loaded trajectory.
   *
   * @return The first pose in the first trajectory, or {@link Pose2d#kZero} if
   *         none
   */
  public Pose2d getStartPose() {
    if (trajectory_list_.isEmpty() || trajectory_list_.get(0).length == 0) {
      return Pose2d.kZero;
    }
    return trajectory_list_.get(0)[0];
  }

  /**
   * Get the full path as an array of Pose2d, flipped for alliance if needed
   * 
   * @param alliance The alliance color
   * @return Array of Pose2d representing the path
   */
  public Pose2d[] getPath(Alliance alliance) {
    // Flip the trajectory for red alliance
    if (alliance == Alliance.Red) {
      return trajectory_list_.stream()
          .flatMap(Arrays::stream)
          .map(AllianceFlipUtil::apply)
          .toArray(Pose2d[]::new);
    }
    return trajectory_list_.stream()
        .flatMap(Arrays::stream)
        .toArray(Pose2d[]::new);
  }

}
