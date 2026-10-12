package com.marswars.auto;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Checks, chooses, prepares and runs the autonomous routine.
 *
 * <ul>
 *   <li><b>Checking:</b> at boot every registered {@link AutoRoutine} is prepared once on a
 *       background thread (for behavior-tree autos: the XML, every path file and every event name).
 *       Each one that loads is added to the dashboard "Auto Chooser" as soon as its check finishes;
 *       one that fails is left out and raises an alert. Robot code never waits for this.
 *   <li><b>Choosing:</b> "Selected Auto Path" previews the selected routine's paths. The selection
 *       is a logged input ({@code LoggedDashboardChooser}), so log replay runs the auto that ran in
 *       the match.
 *   <li><b>Preparing:</b> when the selection or the alliance changes, the selected routine's
 *       {@link AutoRoutine#prepare} (reading files, loading and flipping paths) runs on the
 *       background thread and the step it returns runs on the robot thread ({@link AutoPreparer}).
 *       Nothing is prepared in {@code autonomousInit}: the routine is ready before the match.
 *   <li><b>Running:</b> {@link #start()} in {@code autonomousInit}; {@link #periodic()} every loop
 *       steps it (a behavior tree is ticked here, a command auto runs in the {@code
 *       CommandScheduler}); it is stopped when it finishes, when autonomous ends, or by {@link
 *       #stop()}. If the selected routine is not prepared yet when autonomous starts (the selection
 *       changed a moment before), it starts as soon as its preparation lands.
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
      new AutoPreparer(AutoPreparer.backgroundThread(), this::onPrepared);

  // Boot check: routines waiting to be checked, the one being checked, and those that failed.
  private final Deque<AutoRoutine> unchecked_ = new ArrayDeque<>();
  private AutoRoutine checking_ = null;
  private final List<String> rejected_ = new ArrayList<>();
  private final Alert rejected_alert_ = new Alert("", AlertType.kError);
  private int checked_count_ = 0;
  private long check_start_ns_ = 0;

  private boolean pending_auto_update_ = true;
  private boolean start_pending_ = false;
  private Optional<Alliance> current_alliance_ = Optional.empty();
  private AutoRoutine running_ = null;

  private AutoManager() {
    auto_chooser_.addDefaultOption(do_nothing_.getName(), do_nothing_);
    registered_names_.add(do_nothing_.getName());
    // A flag, not work: the callback runs inside the logger's periodic, so keep it trivial.
    auto_chooser_.onChange(auto -> pending_auto_update_ = true);
    SmartDashboard.putData("Selected Auto Path", auto_display);
  }

  /**
   * Registers routines. Each is checked in the background and added to the chooser once it loads;
   * a duplicate name keeps the first and reports an error.
   */
  public void registerAutos(AutoRoutine... autos) {
    for (AutoRoutine auto : autos) {
      String name = auto.getName();
      if (!registered_names_.add(name)) {
        DriverStation.reportError(
            "Duplicate auto name '" + name + "': the second one is not in the chooser", false);
        continue;
      }
      if (unchecked_.isEmpty() && checking_ == null) {
        check_start_ns_ = System.nanoTime();
        checked_count_ = 0;
      }
      unchecked_.add(auto);
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
   * file and every routine reloads its paths from disk. Use after copying new files to the robot.
   */
  public void requestReload() {
    AutoTrajectories.clearCache();
    pending_auto_update_ = true;
  }

  /** The routine selected on the dashboard (in replay, the one the log says was selected). */
  public AutoRoutine getSelectedAuto() {
    AutoRoutine selected = auto_chooser_.get();
    if (selected == null) { // not (yet) an option: still being checked, broken, or renamed
      return do_nothing_;
    }
    return selected;
  }

  /** The routine running now, if any. */
  public Optional<AutoRoutine> getRunningAuto() {
    return Optional.ofNullable(running_);
  }

  /** True while the boot check of the registered routines is still going. */
  public boolean isChecking() {
    return checking_ != null || !unchecked_.isEmpty();
  }

  /**
   * Every loop, from robotPeriodic(): steps the running routine, checks registered routines, and
   * prepares the selected one.
   */
  public void periodic() {
    stepRunning();
    preparer_.poll();
    if (!current_alliance_.equals(DriverStation.getAlliance())) {
      current_alliance_ = DriverStation.getAlliance();
      pending_auto_update_ = true;
      DataLogManager.log("Alliance changed, triggering auto update");
    }
    // One preparation at a time, never while a routine runs. The boot check goes first.
    if (!preparer_.isBusy() && running_ == null) {
      if (!unchecked_.isEmpty()) {
        checking_ = unchecked_.poll();
        preparer_.submit(checking_, isRed());
      } else if (pending_auto_update_ && (current_alliance_.isPresent() || start_pending_)) {
        pending_auto_update_ = false;
        preparer_.submit(getSelectedAuto(), isRed());
      }
    }
    startPendingAuto();
  }

  /**
   * Starts the selected routine (from {@code autonomousInit}). It never loads anything here: the
   * routine was prepared while disabled. If it isn't prepared for this alliance yet, it starts as
   * soon as its preparation lands.
   */
  public void start() {
    stop();
    AutoRoutine auto = getSelectedAuto();
    if (isChecking() || !preparer_.isPrepared(auto, isRed())) {
      DriverStation.reportError(
          "Auto "
              + auto.getName()
              + " is not prepared for "
              + (isRed() ? "red" : "blue")
              + " yet; it starts when its preparation finishes",
          false);
      pending_auto_update_ = true;
      start_pending_ = true;
      return;
    }
    begin(auto);
  }

  /** Stops the running routine, if any (from {@code disabledInit} / {@code teleopInit}). */
  public void stop() {
    start_pending_ = false;
    if (running_ != null) {
      DataLogManager.log("Stopping auto routine: " + running_.getName());
      running_.stop(true);
      running_ = null;
    }
  }

  private void begin(AutoRoutine auto) {
    DataLogManager.log("Starting auto routine: " + auto.getName());
    running_ = auto;
    auto.start();
  }

  /** Starts an auto whose start was deferred, once it is prepared (only while in autonomous). */
  private void startPendingAuto() {
    if (!start_pending_) {
      return;
    }
    if (!DriverStation.isAutonomousEnabled()) {
      start_pending_ = false;
      return;
    }
    AutoRoutine auto = getSelectedAuto();
    if (running_ == null && !isChecking() && !preparer_.isBusy()
        && preparer_.isPrepared(auto, isRed())) {
      start_pending_ = false;
      DataLogManager.log("Auto " + auto.getName() + " prepared; starting it late");
      begin(auto);
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

  /** Robot thread, after each preparation is put in use. */
  private void onPrepared(AutoRoutine auto) {
    if (auto == checking_) {
      checking_ = null;
      checked_count_++;
      if (auto.isLoaded()) {
        auto_chooser_.addOption(auto.getName(), auto);
      } else {
        rejected_.add(auto.getName());
        rejected_alert_.setText("Autos not in the chooser (they failed to load): "
            + String.join(", ", rejected_));
        rejected_alert_.set(true);
      }
      // the dashboard may already have this option selected (a reboot keeps the selection)
      pending_auto_update_ = true;
      if (!isChecking()) {
        DataLogManager.log(
            "Checked "
                + checked_count_
                + " autos in "
                + (System.nanoTime() - check_start_ns_) / 1_000_000
                + " ms; not in the chooser: "
                + rejected_);
      }
    }
    showPath(auto);
  }

  private boolean isRed() {
    return current_alliance_.map(a -> a == Alliance.Red).orElse(false);
  }

  /** Shows a freshly prepared routine's paths, if it is the one selected. */
  private void showPath(AutoRoutine auto) {
    if (auto == getSelectedAuto()) {
      auto_display.getObject("Auto Path").setPoses(auto.getPath());
    }
  }
}
