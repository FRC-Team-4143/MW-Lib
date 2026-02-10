package com.marswars.auto;

import java.util.Optional;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final Trigger ds_trigger_ = new Trigger(DriverStation::isDSAttached);

  private AutoManager() {
    // Create the auto chooser
    auto_chooser_ = new SendableChooser<Auto>();
    // Set default option to not move and wait 30 seconds
    Auto doNothing = new Auto();
    doNothing.addCommands(Commands.waitSeconds(30));
    auto_chooser_.setDefaultOption("Do_Nothing", doNothing);
    
    // Bind a callback on selected change to display auto
    auto_chooser_.onChange((auto) -> {
      visualizeAuto(auto);
    });

    // Trigger to detect driver station attachment
    ds_trigger_.onTrue(Commands.runOnce(()-> {
      Auto selected_auto = getSelectedAuto();
      visualizeAuto(selected_auto);
    }));

    // Put the auto chooser and auto display on the dashboard once during initialization
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
   * Get the selected auto routine
   * 
   * @return The selected Auto as a command sequence
   */
  public Auto getSelectedAuto() {
    Auto auto = auto_chooser_.getSelected();
    DataLogManager.log("Selected auto routine: " + auto.getClass().getSimpleName());
    return auto;
  }

  /**
   * Displays the currently selected auto path on the dashboard field.
   *
   * @param auto The auto routine whose path should be visualized
   */
  public void visualizeAuto(Auto auto) {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if(alliance.isEmpty()) {
      DataLogManager.log("Alliance not yet determined; cannot visualize auto path");
      return;
    }

    auto_display.getObject("Auto Path").setPoses(auto.getPath(alliance.get()));
    // No need to call putData again - the Field2d object is already on SmartDashboard
    // and will automatically update when we change its poses
  }

}
