package com.marswars.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Base class for command-based autonomous routines: a {@code SequentialCommandGroup} that loads
 * Choreo trajectories and exposes common path utilities. As an {@link AutoRoutine} it runs through
 * the {@code CommandScheduler}: {@link #start()} schedules it, {@link #stop} cancels it. Behavior-tree
 * autos ({@link BehaviorTreeAuto}) don't use this class; they run straight from the robot loop.
 */
public class Auto extends SequentialCommandGroup implements AutoRoutine {
  private final AutoTrajectories paths_ = new AutoTrajectories();

  /** Trajectory info for visualization / path following (null values until loaded). */
  protected final Map<String, ChoreoTrajectory> trajectories_ = paths_.map();

  /** Creates a new autonomous routine container with a default name. */
  public Auto() {
    this.setName(getClass().getSimpleName());
  }

  /**
   * Registers a trajectory to be part of the auto. The actual trajectory will be
   * loaded once the auto is selected, allowing for pre-caching of trajectories
   * without loading them all at once during initialization.
   *
   * @param name The name of the trajectory to load
   */
  protected void loadTrajectory(String name) {
    paths_.register(name);
  }

  /**
   * Loads all registered trajectories from Choreo and processes them for use in
   * the auto. Called when the auto is selected, so paths are ready before it runs.
   *
   * @param is_red_alliance true if the robot is on the red alliance, false for
   *                        blue (used for flipping trajectories)
   */
  public void cacheTrajetories(boolean is_red_alliance) {
    paths_.loadAll(is_red_alliance);
  }

  /**
   * Get a loaded trajectory by name.
   *
   * @param name The name of the trajectory
   * @return The typed Choreo trajectory, looked up when the supplier is called
   */
  protected Supplier<ChoreoTrajectory> getTrajectory(String name) {
    return paths_.get(name);
  }

  /**
   * Gets the starting pose for the first loaded trajectory.
   *
   * @return The first pose in the first trajectory, or {@link Pose2d#kZero} if none
   */
  @Override
  public Pose2d getStartPose() {
    return paths_.startPose();
  }

  /**
   * Get the full path as an array of Pose2d
   *
   * @return Array of Pose2d representing the path
   */
  @Override
  public Pose2d[] getPath() {
    return paths_.path();
  }

  // ------------------------------------------------------------------ AutoRoutine

  /** Loads the paths; safe off the main thread (it only reads files). */
  @Override
  public void prepare(boolean is_red_alliance) {
    cacheTrajetories(is_red_alliance);
  }

  @Override
  public void start() {
    CommandScheduler.getInstance().schedule(this);
  }

  @Override
  public void periodic() {
    // the CommandScheduler runs the command group
  }

  @Override
  public void stop(boolean interrupted) {
    if (isScheduled()) {
      cancel();
    }
  }

  @Override
  public boolean isRunning() {
    return isScheduled();
  }
}
