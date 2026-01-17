package com.marswars.subsystem;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import com.marswars.logging.GitLogger;
import com.marswars.util.ConstantsLoader;
import java.util.ArrayList;
import java.util.List;

public abstract class SubsystemManager {
    private static final String subsystems_key_ = "disabled_subsystems";

    protected ArrayList<MwSubsystemBase> subsystems;
    protected Notifier loopThread;
    protected boolean log_init = false;

    private static StringPublisher robot_name_pub_ =
            NetworkTableInstance.getDefault().getStringTopic("/Metadata/ROBOT_NAME").publish();

    protected static List<String> disabled_subsystems_;

    public static List<String> getEnabledSubsystems() {
        return disabled_subsystems_;
    }

    public SubsystemManager(Object build_constants) {
        // Initialize the subsystem list
        subsystems = new ArrayList<>();

        // setup all logging
        if (RobotBase.isSimulation()) {
            DogLog.setOptions(new DogLogOptions().withNtPublish(true).withCaptureDs(true));
        } else {
            DogLog.setOptions(new DogLogOptions().withCaptureNt(true).withCaptureDs(true));
            DogLog.setPdh(null);
        }
        DogLog.setEnabled(true);

        // Log robot metadata
        GitLogger.logGitData(build_constants);
        robot_name_pub_.set(ConstantsLoader.getInstance().getRobotName());

        // Handle disabling subsystems
        disabled_subsystems_ = ConstantsLoader.getInstance().getStringList(subsystems_key_);
        DataLogManager.log("Disabling subsystems: " + disabled_subsystems_.toString());
    }

    public void registerSubsystem(MwSubsystemBase system) {
        if (disabled_subsystems_.contains(system.getName())) {
            DataLogManager.log(
                    "Registered disabled subsystem: " + system.getClass().getSimpleName());
        } else {
            subsystems.add(system);
        }
    }

    /** Preform the control loop for all subsystems */
    public void doControlLoop() {
        // For each subsystem run its update loop
        for (MwSubsystemBase subsystem : subsystems) {
            try {
                DogLog.time(subsystem.getSubsystemKey() + "/loop_time");

                List<SubsystemIoBase> ios = subsystem.getIos();

                // Run the subsystem update loop
                double timestamp = Timer.getFPGATimestamp();

                for (SubsystemIoBase io : ios) {
                    io.readInputs(timestamp);
                }

                subsystem.update(timestamp);

                for (SubsystemIoBase io : ios) {
                    io.writeOutputs(timestamp);
                    io.logData();
                }

                DogLog.timeEnd(subsystem.getSubsystemKey() + "/loop_time");
            } catch (Exception e) {
                DataLogManager.log(
                        " Failed to run update loop for "
                                + subsystem.getClass().getCanonicalName());
                e.printStackTrace();
            }
        }
    }

    /**
     * If subsystems all need to be reset before a robot mode change, call this function to cleanly
     * handle resetting them together. If only one subsystem needs to be reset, that can be accessed
     * through the getInstance method.
     */
    public void reset() {
        for (MwSubsystemBase subsystem : subsystems) {
            subsystem.reset();
        }
    }
}
