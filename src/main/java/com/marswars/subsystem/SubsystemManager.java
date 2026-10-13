package com.marswars.subsystem;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Notifier;

import com.marswars.logging.BatteryLogger;
import com.marswars.logging.MwLog;
import com.marswars.util.RobotIdentity;
import java.util.ArrayList;
import java.util.List;

public abstract class SubsystemManager {
    protected ArrayList<MwSubsystemBase> subsystems;
    protected Notifier loopThread;
    protected boolean log_init = false;
    private final boolean start_logging_;

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
        this(build_constants, disabled_subsystems, true);
    }

    /**
     * @param start_logging false only in unit tests: AdvantageKit exits the JVM if it is started
     *     outside a LoggedRobot, and battery logging needs the PWF native library
     */
    SubsystemManager(
            Object build_constants, List<String> disabled_subsystems, boolean start_logging) {
        // Initialize the subsystem list
        subsystems = new ArrayList<>();

        // Start AdvantageKit logging (idempotent; sets up receivers + Logger.start,
        // and records/publishes git & build metadata)
        start_logging_ = start_logging;
        if (start_logging) {
            MwLog.init(build_constants);
        }

        robot_name_pub_.set(RobotIdentity.getInstance().getRobotName());
        if (start_logging) {
            BatteryLogger.logBatteryData();
        }

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

        // Let a coordinator (see MwSuperStateMachine) request subsystem states first, so its
        // requests take effect in this same loop
        update(MwLog.timestampSeconds());

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
        if (start_logging_) {
            BatteryLogger.logBatteryData();
        }
    }

    /**
     * The manager's own update, named like a subsystem's: runs once per loop after tunables are
     * polled and before any subsystem updates. Does nothing by default; {@link MwSuperStateMachine}
     * overrides it to run the robot-level state machine.
     *
     * @param timestamp the loop timestamp in seconds
     */
    protected void update(double timestamp) {}

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
