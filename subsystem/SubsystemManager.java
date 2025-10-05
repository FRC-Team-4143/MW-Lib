package frc.mw_lib.subsystem;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.mw_lib.logging.GitLogger;
import frc.mw_lib.util.ConstantsLoader;
import java.util.ArrayList;
import java.util.List;

public abstract class SubsystemManager {
  private static final String subsystems_key_ = "disabled_subsystems";

  protected ArrayList<MWSubsystemBase> subsystems;
  protected Notifier loopThread;
  protected boolean log_init = false;

  protected static List<String> disabled_subsystems_;

  public static List<String> getEnabledSubsystems() {
    return disabled_subsystems_;
  }

  public SubsystemManager() {
    // Initialize the subsystem list
    subsystems = new ArrayList<>();

    disabled_subsystems_ = ConstantsLoader.getInstance().getStringList(subsystems_key_);
    DataLogManager.log("Disabling subsystems: " + disabled_subsystems_.toString());

    GitLogger.logGitData();
  }

  public void registerSubsystem(MWSubsystemBase system) {
    if (disabled_subsystems_.contains(system.getName())) {
      DataLogManager.log("Registered disabled subsystem: " + system.getClass().getSimpleName());
    } else {
      subsystems.add(system);
    }
  }

  public void doControlLoop() {
    // For each subsystem get incoming data
    for (MWSubsystemBase subsystem : subsystems) {
      try {
        double timestamp = Timer.getFPGATimestamp();

        DogLog.time(subsystem.getSubsystemKey() + "/loop_time");

        subsystem.getIo().readInputs(timestamp);

        subsystem.update(timestamp);

        subsystem.getIo().writeOutputs(timestamp);

        DogLog.timeEnd(subsystem.getSubsystemKey() + "/loop_time");

        subsystem.logData();
        subsystem.getIo().logData();

      } catch (Exception e) {
        e.printStackTrace();
        DataLogManager.log(subsystem.getClass().getCanonicalName() + "failed to read inputs");
      }
    }
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
