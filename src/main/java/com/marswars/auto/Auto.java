package com.marswars.auto;

import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Function;
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
  public void cacheTrajetories(boolean is_red_alliance) {
    cacheTrajetories(is_red_alliance, name -> loadFromChoreo(name));
  }

  /**
   * Loads a .traj of either sample type. The type argument is only a witness for Choreo's generic
   * signature (erased at runtime); Choreo picks Swerve or Differential samples from the file.
   */
  private static Trajectory<?> loadFromChoreo(String name) {
    return choreo.Choreo.<choreo.trajectory.SwerveSample>loadTrajectory(name).get();
  }

  /**
   * Loads registered trajectories with the given loader. Package-private so tests can supply
   * in-memory trajectories instead of reading deploy files.
   */
  void cacheTrajetories(boolean is_red_alliance, Function<String, Trajectory<?>> loader) {
    synchronized (trajectories_) {
      for (var entry : trajectories_.entrySet()) {
        // Choreo parses the .traj by its sampleType (Swerve or Differential), so keep the
        // sample type open here and let ChoreoTrajectory record what it actually got.
        Trajectory<?> traj = loader.apply(entry.getKey());
        entry.setValue(ChoreoTrajectory.ofUnknown(traj, is_red_alliance));
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
    Pose2d[] poses = trajectories_.values().iterator().next().getPoses();
    return poses.length == 0 ? Pose2d.kZero : poses[0];
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
          .flatMap(t -> Arrays.stream(t.getPoses()))
          .toArray(Pose2d[]::new);
    }
  }

}
