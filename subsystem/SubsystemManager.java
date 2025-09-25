package frc.mw_lib.subsystem;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.mw_lib.util.ConstantsLoader;
import java.util.ArrayList;
import java.util.List;

public abstract class SubsystemManager {
  private static final String subsystems_key_ = "subsystems";

  protected ArrayList<MWSubsystemBase> subsystems;
  protected Notifier loopThread;
  protected boolean log_init = false;

  protected static List<String> enabled_systems_;

  public static List<String> getEnabledSubsystems() {
    if (enabled_systems_ == null) {
      // Determine removable subsystems to load
      enabled_systems_ = ConstantsLoader.getInstance().getStringList(subsystems_key_);
      DataLogManager.log("Expecting subsystems: " + enabled_systems_.toString());
    }

    return enabled_systems_;
  }

  public SubsystemManager() {
    // Initialize the subsystem list
    subsystems = new ArrayList<>();

    // create the thread to loop the subsystems and mark as daemon thread so
    // the robot program can properly stop
    loopThread = new Notifier(this::doControlLoop);
  }

  public void registerSubsystem(MWSubsystemBase system) {
    if (!enabled_systems_.contains(system.getName())) {
      DataLogManager.log("Registered disabled subsystem: " + system.getClass().getSimpleName());
    } else {
      subsystems.add(system);
    }
  }

  private void doControlLoop() {

    // For each subsystem get incoming data
    double timestamp = Timer.getFPGATimestamp();
    for (MWSubsystemBase subsystem : subsystems) {
      try {
        subsystem.getIo().readInputs(timestamp);
      } catch (Exception e) {
        e.printStackTrace();
        DataLogManager.log(subsystem.getClass().getCanonicalName() + "failed to read inputs");
      }
    }

    // Now update the logic for each subsystem to allow I/O to relax
    timestamp = Timer.getFPGATimestamp();
    for (MWSubsystemBase subsystem : subsystems) {
      try {
        subsystem.updateLogic(timestamp);
      } catch (Exception e) {
        e.printStackTrace();
        DataLogManager.log(subsystem.getClass().getCanonicalName() + "failed to update logic");
      }
    }

    // Finally write the outputs to the actuators
    timestamp = Timer.getFPGATimestamp();
    for (MWSubsystemBase subsystem : subsystems) {
      try {
        subsystem.getIo().writeOutputs(timestamp);
      } catch (Exception e) {
        e.printStackTrace();
        DataLogManager.log(subsystem.getClass().getCanonicalName() + "failed to write outputs");
      }
    }
  }

  /** Completes the subsystem registration process and begins calling each subsystem in a loop */
  protected void completeRegistration() {
    loopThread.startPeriodic(.01);
  }

  /**
   * If subsystems all need to be reset before a robot mode change, call this function to cleanly
   * handle resetting them together. If only one subsystem needs to be reset, that can be accessed
   * through the getInstance method.
   */
  public void reset() {
    for (MWSubsystemBase subsystem : subsystems) {
      subsystem.reset();
    }
  }
}
