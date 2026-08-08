package com.marswars.subsystem;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Notifier;

import com.marswars.logging.BatteryLogger;
import com.marswars.logging.GitLogger;
import com.marswars.logging.MwLog;
import com.marswars.util.RobotIdentity;
import java.util.ArrayList;
import java.util.List;

public abstract class SubsystemManager {
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
        this(build_constants, List.of());
    }

    public SubsystemManager(Object build_constants, List<String> disabled_subsystems) {
        // Initialize the subsystem list
        subsystems = new ArrayList<>();

        // Start AdvantageKit logging (idempotent; sets up receivers + Logger.start)
        MwLog.init(build_constants);

        // Log robot metadata to NT (GitLogger continues using NT publishers directly)
        GitLogger.logGitData(build_constants);
        robot_name_pub_.set(RobotIdentity.getInstance().getRobotName());
        BatteryLogger.logBatteryData();

        // Handle disabling subsystems
        disabled_subsystems_ = disabled_subsystems;
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
        // Poll tunables and fire onChange consumers before subsystem logic runs
        MwLog.periodic();

        // For each subsystem run its update loop
        for (MwSubsystemBase subsystem : subsystems) {
            try {
                MwLog.time(subsystem.getSubsystemKey() + "/loop_time");

                List<SubsystemIoBase> ios = subsystem.getIos();

                // Deterministic timestamp — sourced from the log during replay
                double timestamp = MwLog.timestampSeconds();

                for (SubsystemIoBase io : ios) {
                    io.readInputs(timestamp);
                }

                subsystem.update(timestamp);

                for (SubsystemIoBase io : ios) {
                    io.writeOutputs(timestamp);
                    io.logData();
                }

                MwLog.timeEnd(subsystem.getSubsystemKey() + "/loop_time");
            } catch (Exception e) {
                DataLogManager.log(
                        " Failed to run update loop for "
                                + subsystem.getClass().getCanonicalName());
                e.printStackTrace();
            }
        }
        // Log battery data every loop
        BatteryLogger.logBatteryData();
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
