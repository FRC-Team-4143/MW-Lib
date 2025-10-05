package frc.mw_lib.subsystem;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
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

  private static StringPublisher robot_name_pub_ = NetworkTableInstance.getDefault()
      .getStringTopic("/Metadata/ROBOT_NAME").publish();

  protected static List<String> disabled_subsystems_;

  public static List<String> getEnabledSubsystems() {
    return disabled_subsystems_;
  }

  public SubsystemManager() {
    // Initialize the subsystem list
    subsystems = new ArrayList<>();

    // setup all logging
    if (RobotBase.isSimulation()) {
      DogLog.setOptions(new DogLogOptions().withNtPublish(true).withCaptureDs(true));
    } else {
      DogLog.setOptions(new DogLogOptions().withCaptureNt(true).withCaptureDs(true));
      DogLog.setPdh(new PowerDistribution());
    }
    DogLog.setEnabled(true);

    // Log robot metadata
    GitLogger.logGitData();
    robot_name_pub_.set(ConstantsLoader.getInstance().getRobotName());

    // Handle disabling subsystems
    disabled_subsystems_ = ConstantsLoader.getInstance().getStringList(subsystems_key_);
    DataLogManager.log("Disabling subsystems: " + disabled_subsystems_.toString());
  }

  public void registerSubsystem(MWSubsystemBase system) {
    if (disabled_subsystems_.contains(system.getName())) {
      DataLogManager.log("Registered disabled subsystem: " + system.getClass().getSimpleName());
    } else {
      subsystems.add(system);
    }
  }

  public void doControlLoop() {
    // For each subsystem run its update loop
    for (MWSubsystemBase subsystem : subsystems) {
      try {
        DogLog.time(subsystem.getSubsystemKey() + "/loop_time");

        // Run the subsystem update loop
        double timestamp = Timer.getFPGATimestamp();
        subsystem.getIo().readInputs(timestamp);
        subsystem.update(timestamp);
        subsystem.getIo().writeOutputs(timestamp);
        subsystem.getIo().logData();

        DogLog.timeEnd(subsystem.getSubsystemKey() + "/loop_time");
      } catch (Exception e) {
        DataLogManager.log(" Failed to run update loop for " + subsystem.getClass().getCanonicalName());
        e.printStackTrace();
      }
    }
  }

  /**
   * If subsystems all need to be reset before a robot mode change, call this
   * function to cleanly handle resetting them together. If only one subsystem
   * needs to be reset, that can be accessed through the getInstance method.
   */
  public void reset() {
    for (MWSubsystemBase subsystem : subsystems) {
      subsystem.reset();
    }
  }
}
