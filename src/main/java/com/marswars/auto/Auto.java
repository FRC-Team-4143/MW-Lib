package com.marswars.auto;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.command2.SequentialCommandGroup;

import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Base class for autonomous routines that load Choreo trajectories and expose
 * common path utilities.
 */
public class Auto extends SequentialCommandGroup {
  // Trajectory info for visualization / path following
  protected final Map<String, ChoreoTrajectory> trajectories_ = Collections.synchronizedMap(new LinkedHashMap<>());

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
   * @param is_red_alliance true if the robot is on the red alliance, false for
   *                        blue (used for flipping trajectories)
   */
  @SuppressWarnings("unchecked")
  public void cacheTrajetories(boolean is_red_alliance) {
    synchronized (trajectories_) {
      for (var entry : trajectories_.entrySet()) {
        String name = entry.getKey();
        // request the choreo trajectory to be loaded
        Trajectory<SwerveSample> traj = (Trajectory<SwerveSample>) choreo.Choreo.loadTrajectory(name).get();

        // load the trajectory with event markers into our typed ChoreoTrajectory class
        // and store it
        ChoreoTrajectory choreoTraj = new ChoreoTrajectory(traj, is_red_alliance);
        entry.setValue(choreoTraj);

      }
    }
  }

  /**
   * Get a loaded trajectory by name.
   *
   * @param name The name of the trajectory
   * @return The typed Choreo trajectory
   * 
   * @throws IllegalStateException if the trajectory has not been loaded yet (i.e.
   *                               cacheTrajectories() has not been called)
   */
  protected Supplier<ChoreoTrajectory> getTrajectory(String name) {
    return () -> {
       ChoreoTrajectory traj = trajectories_.get(name);
       if (traj == null) {
         throw new IllegalStateException("Trajectory " + name
             + " has not been loaded yet. Make sure to call cacheTrajectories() after selecting the auto.");
       }
       return traj;
    };
  }

  /**
   * Gets the starting pose for the first loaded trajectory.
   *
   * @return The first pose in the first trajectory, or {@link Pose2d#kZero} if
   *         none
   */
  public Pose2d getStartPose() {
    if (trajectories_.isEmpty() || trajectories_.values().iterator().next() == null) {
      return Pose2d.kZero;
    }
    SwerveSample first = trajectories_.values().iterator().next().getTrajectory().samples().get(0);
    return new Pose2d(first.x, first.y, new Rotation2d(first.heading));
  }

  /**
   * Get the full path as an array of Pose2d
   *
   * @return Array of Pose2d representing the path
   */
  public Pose2d[] getPath() {
    synchronized (trajectories_) {
      return trajectories_.values().stream()
          .filter(t -> t != null)
          .flatMap(t -> t.getTrajectory().samples().stream())
          .map(s -> new Pose2d(s.x, s.y, new Rotation2d(s.heading)))
          .toArray(Pose2d[]::new);
    }
  }

}
