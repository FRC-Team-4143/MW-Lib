package com.marswars.auto;

import java.util.Optional;


import org.wpilib.system.DataLogManager;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.Trigger;

/**
 * Singleton manager for autonomous routine selection and visualization.
 */
public class AutoManager {
  // Singleton pattern
  private static AutoManager instance_ = null;

  /**
   * Gets the singleton AutoManager instance.
   *
   * @return the AutoManager instance
   */
  public static AutoManager getInstance() {
    if (instance_ == null) {
      instance_ = new AutoManager();
    }
    return instance_;
  }

  private final SendableChooser<Auto> auto_chooser_;
  private final Field2d auto_display = new Field2d();
  private boolean pending_auto_update_ = true;
  private Optional<Alliance> current_alliance_ = Optional.empty();

  private AutoManager() {
    // Create the auto chooser
    auto_chooser_ = new SendableChooser<Auto>();
    // Set default option to not move and wait 30 seconds
    Auto doNothing = new Auto();
    doNothing.addCommands(Commands.waitSeconds(30));
    auto_chooser_.setDefaultOption("Do_Nothing", doNothing);

    // Bind a callback on selected change to display auto.
    // Use a flag instead of calling directly to avoid ConcurrentModificationException
    // caused by modifying SmartDashboard's map while updateValues() is iterating it.
    auto_chooser_.onChange((auto) -> {
      pending_auto_update_ = true;
    });

    // Put the auto chooser and auto display on the dashboard once during
    // initialization
    SmartDashboard.putData("Auto Chooser", auto_chooser_);
    SmartDashboard.putData("Selected Auto Path", auto_display);
  }

  /**
   * Register multiple auto routines to the chooser
   * 
   * @param autos Varargs of Auto routines to register
   */
  public void registerAutos(Auto... autos) {
    for (Auto auto : autos) {
      auto_chooser_.addOption(auto.getClass().getSimpleName(), (Auto) auto);
    }
  }

  /**
   * Must be called from robotPeriodic(). Processes any pending auto selection
   * changes outside of SmartDashboard's updateValues() iteration to avoid
   * ConcurrentModificationException.
   */
  public void periodic() {
    if(current_alliance_ != MatchState.getAlliance()) {
      current_alliance_ = MatchState.getAlliance();
      pending_auto_update_ = true;
      DataLogManager.log("Alliance changed, Triggering auto update");
    }

    if (pending_auto_update_ && current_alliance_.isPresent()) {
      pending_auto_update_ = false;
      onSelectedAutoChange();
    }
  }

  /**
   * Get the selected auto routine
   * 
   * @return The selected Auto as a command sequence
   */
  public Auto getSelectedAuto() {
    Auto auto = auto_chooser_.getSelected();
    DataLogManager.log("Selected auto routine: " + auto.getClass().getSimpleName());
    return auto;
  }

  public void onSelectedAutoChange() {
    // determine what auto we select
    Auto selected_auto = getSelectedAuto();

    // determine our alliance for path flipping
    Optional<Alliance> alliance = MatchState.getAlliance();

    if (alliance.isEmpty()) {
      DataLogManager.log("Alliance not yet determined; cannot visualize auto path");
      return;
    }

    // hot load its paths
    selected_auto.cacheTrajetories(alliance.get() == Alliance.RED);

    // update the dashboard with the new path
    auto_display.getObject("Auto Path").setPoses(selected_auto.getPath());
  }
}