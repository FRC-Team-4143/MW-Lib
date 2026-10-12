package com.marswars.auto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Chooses, prepares and runs the autonomous routine.
 *
 * <ul>
 *   <li><b>Choosing:</b> every registered {@link AutoRoutine} is an option in the dashboard "Auto
 *       Chooser"; "Selected Auto Path" previews its paths. The selection is a logged input
 *       ({@code LoggedDashboardChooser}), so log replay runs the auto that ran in the match.
 *   <li><b>Preparing:</b> when the selection or the alliance changes, the selected routine's
 *       {@link AutoRoutine#prepare} (reading files, loading and flipping paths) runs on a background
 *       thread and the step it returns runs on the robot thread ({@link AutoPreparer}).
 *   <li><b>Running:</b> {@link #start()} in {@code autonomousInit}; {@link #periodic()} every loop
 *       steps it (a behavior tree is ticked here, a command auto runs in the {@code
 *       CommandScheduler}); it is stopped when it finishes, when autonomous ends, or by {@link
 *       #stop()}.
 * </ul>
 *
 * <p>Call {@link #periodic()} every loop from {@code robotPeriodic()}, after {@code
 * CommandScheduler.run()} and before the robot's {@code doControlLoop()}, so the states a tree asks
 * for are acted on in the same loop.
 */
public class AutoManager {
  private static AutoManager instance_ = null;

  /** Gets the singleton AutoManager instance. */
  public static AutoManager getInstance() {
    if (instance_ == null) {
      instance_ = new AutoManager();
    }
    return instance_;
  }

  private final LoggedDashboardChooser<AutoRoutine> auto_chooser_ =
      new LoggedDashboardChooser<>("Auto Chooser");
  private final AutoRoutine do_nothing_ = AutoRoutine.none("Do_Nothing");
  private final Field2d auto_display = new Field2d();
  private final Set<String> registered_names_ = new HashSet<>();
  private final AutoPreparer preparer_ =
      new AutoPreparer(AutoPreparer.backgroundThread(), this::showPath);

  private boolean pending_auto_update_ = true;
  private Optional<Alliance> current_alliance_ = Optional.empty();
  private AutoRoutine running_ = null;

  private AutoManager() {
    auto_chooser_.addDefaultOption(do_nothing_.getName(), do_nothing_);
    registered_names_.add(do_nothing_.getName());
    // A flag, not work: the callback runs inside the logger's periodic, so keep it trivial.
    auto_chooser_.onChange(auto -> pending_auto_update_ = true);
    SmartDashboard.putData("Selected Auto Path", auto_display);
  }

  /** Adds routines to the chooser; a duplicate name keeps the first and reports an error. */
  public void registerAutos(AutoRoutine... autos) {
    for (AutoRoutine auto : autos) {
      String name = auto.getName();
      if (!registered_names_.add(name)) {
        DriverStation.reportError(
            "Duplicate auto name '" + name + "': the second one is not in the chooser", false);
        continue;
      }
      auto_chooser_.addOption(name, auto);
    }
  }

  /** Adds a list of routines (e.g. from {@link BehaviorTreeAuto#loadAll}). */
  public void registerAutos(List<? extends AutoRoutine> autos) {
    registerAutos(autos.toArray(new AutoRoutine[0]));
  }

  /** Adds any WPILib command as a routine (see {@link AutoRoutine#of}). */
  public void registerAuto(String name, Command command) {
    registerAutos(AutoRoutine.of(name, command));
  }

  /**
   * Prepare the selected routine again on the next {@link #periodic()}: XML autos re-read their
   * file and every routine reloads its paths. Use after copying new files to the robot.
   */
  public void requestReload() {
    pending_auto_update_ = true;
  }

  /** The routine selected on the dashboard (in replay, the one the log says was selected). */
  public AutoRoutine getSelectedAuto() {
    AutoRoutine selected = auto_chooser_.get();
    if (selected == null) { // a logged name no auto has any more (renamed since the match)
      DataLogManager.log("Selected auto is not registered; running " + do_nothing_.getName());
      return do_nothing_;
    }
    return selected;
  }

  /** The routine running now, if any. */
  public Optional<AutoRoutine> getRunningAuto() {
    return Optional.ofNullable(running_);
  }

  /** Every loop, from robotPeriodic(): steps the running routine and prepares the selected one. */
  public void periodic() {
    stepRunning();
    preparer_.poll();
    if (!current_alliance_.equals(DriverStation.getAlliance())) {
      current_alliance_ = DriverStation.getAlliance();
      pending_auto_update_ = true;
      DataLogManager.log("Alliance changed, triggering auto update");
    }
    // One preparation at a time, and never while a routine runs; a change meanwhile waits here.
    if (pending_auto_update_
        && current_alliance_.isPresent()
        && !preparer_.isBusy()
        && running_ == null) {
      pending_auto_update_ = false;
      preparer_.submit(getSelectedAuto(), current_alliance_.get() == Alliance.Red);
    }
  }

  /**
   * Starts the selected routine (from {@code autonomousInit}). If its preparation is still running
   * it waits for it; if it was never prepared for this alliance it prepares it now, on this thread.
   */
  public void start() {
    stop();
    AutoRoutine auto = getSelectedAuto();
    boolean red = DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);
    preparer_.await();
    if (!preparer_.isPrepared(auto, red)) {
      DataLogManager.log(
          "Auto " + auto.getName() + " was not prepared for this alliance; preparing now");
      preparer_.prepareNow(auto, red);
    }
    DataLogManager.log("Starting auto routine: " + auto.getName());
    running_ = auto;
    auto.start();
  }

  /** Stops the running routine, if any (from {@code disabledInit} / {@code teleopInit}). */
  public void stop() {
    if (running_ != null) {
      DataLogManager.log("Stopping auto routine: " + running_.getName());
      running_.stop(true);
      running_ = null;
    }
  }

  /** Steps the running routine; stops it when autonomous ends and drops it when it finishes. */
  private void stepRunning() {
    if (running_ == null) {
      return;
    }
    if (!DriverStation.isAutonomousEnabled()) {
      stop();
      return;
    }
    running_.periodic();
    if (!running_.isRunning()) {
      DataLogManager.log("Auto routine finished: " + running_.getName());
      running_ = null;
    }
  }

  /** Shows a freshly prepared routine's paths, if it is the one selected. */
  private void showPath(AutoRoutine auto) {
    if (auto == getSelectedAuto()) {
      auto_display.getObject("Auto Path").setPoses(auto.getPath());
    }
  }
}
