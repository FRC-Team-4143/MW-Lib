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

/**
 * Base class for autonomous routines that load Choreo trajectories and expose
 * common path utilities.
 */
public class Auto extends SequentialCommandGroup {

  private ArrayList<Pose2d[]> trajectory_list_ = new ArrayList<>();
  protected LinkedHashMap<String, Trajectory<?>> trajectories = new LinkedHashMap<>();

  /** Creates a new autonomous routine container with a default name. */
  public Auto() {
    this.setName(getClass().getSimpleName());
  }

  /** 
   * Load a trajectory by name and store its poses
   * 
   * @param name The name of the trajectory to load
   * 
   * @apiNote You can load multiple trajectories by the same name; their poses will be concatenated
   */
  protected void loadTrajectory(String name) {
    Trajectory<?> traj = choreo.Choreo.loadTrajectory(name).get();

    trajectory_list_.add(traj.getPoses());
    if (!trajectories.containsKey(name)) trajectories.put(name, traj);
  }

  /**
   * Get a loaded trajectory by name.
   *
   * @param name The name of the trajectory
   * @return The typed Choreo trajectory
   */
  @SuppressWarnings("unchecked")
  protected Trajectory<SwerveSample> getTrajectory(String name) {
    return (Trajectory<SwerveSample>) trajectories.get(name);
  }

  /**
   * Gets the starting pose for the first loaded trajectory.
   *
   * @return The first pose in the first trajectory, or {@link Pose2d#kZero} if none
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
    if(alliance == Alliance.Red) {
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
