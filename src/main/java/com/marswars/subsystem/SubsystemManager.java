package com.marswars.subsystem;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.marswars.logging.BatteryLogger;
import com.marswars.logging.MwLog;
import com.marswars.regions.RegionPublisher;
import com.marswars.util.RobotIdentity;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Runs every registered subsystem's control loop.
 *
 * <p>Each subsystem can be switched live between {@link SubsystemMode#ENABLED}, {@link
 * SubsystemMode#DISABLED} and {@link SubsystemMode#TUNING} at {@code
 * /MwLib/Subsystems/<Name>/Mode}, and each of its mechanisms switched off at {@code
 * /MwLib/Subsystems/<Name>/Mechs/<Mech>/Enabled}. Switching costs nothing per loop: a change is
 * queued by an NT listener and applied once at the top of the next loop, which moves the subsystem
 * between the {@code running} list (logic runs) and the {@code parked} list (logic skipped; IO is
 * still read, written and logged). See {@code docs/subsystem-controls.md}.
 */
public abstract class SubsystemManager {
    /** Every registered subsystem, in registration order. */
    protected ArrayList<MwSubsystemBase> subsystems;
    protected Notifier loopThread;
    protected boolean log_init = false;
    private final boolean start_logging_;

    private static StringPublisher robot_name_pub_ =
            NetworkTableInstance.getDefault().getStringTopic("/Metadata/ROBOT_NAME").publish();

    protected static List<String> disabled_subsystems_;

    // ENABLED subsystems run their logic; DISABLED and TUNING ones are parked
    private final List<MwSubsystemBase> running_ = new ArrayList<>();
    private final List<MwSubsystemBase> parked_ = new ArrayList<>();
    private final Map<MwSubsystemBase, SubsystemSwitch> switches_ = new HashMap<>();
    // mode changes from NT listeners, applied on the robot thread at the top of a loop
    private final Queue<Runnable> events_ = new ConcurrentLinkedQueue<>();

    /**
     * @return the names of the subsystems left out at build time (the constructor's {@code
     *     disabled_subsystems} list)
     */
    public static List<String> getDisabledSubsystems() {
        return disabled_subsystems_;
    }

    /**
     * @return the build-time disabled list, despite the name
     * @deprecated returns the <em>disabled</em> list; use {@link #getDisabledSubsystems()}
     */
    @Deprecated
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
            running_.add(system);
            switches_.put(system, new SubsystemSwitch(system, events_, this::sortLists));
        }
    }

    /** Tests: mode changes queued and not yet applied. */
    int pendingModeChanges() {
        return events_.size();
    }

    /** Puts each subsystem in the list for its mode, keeping registration order. */
    private void sortLists() {
        running_.clear();
        parked_.clear();
        for (MwSubsystemBase s : subsystems) {
            (switches_.get(s).mode() == SubsystemMode.ENABLED ? running_ : parked_).add(s);
        }
    }

    private SubsystemSwitch switchFor(MwSubsystemBase subsystem) {
        SubsystemSwitch sw = switches_.get(subsystem);
        if (sw == null) {
            throw new IllegalArgumentException(subsystem.getName() + " is not registered");
        }
        return sw;
    }

    /**
     * Asks for a subsystem's mode from code, the same as a dashboard write. It applies at the top
     * of the next loop.
     *
     * @param subsystem a registered subsystem
     * @param mode the mode
     */
    public void requestMode(MwSubsystemBase subsystem, SubsystemMode mode) {
        switchFor(subsystem).request(mode);
    }

    /**
     * Switches one mechanism on or off from code, the same as a dashboard write. It applies at the
     * top of the next loop.
     *
     * @param subsystem the registered subsystem that owns the mechanism
     * @param mech the mechanism
     * @param enabled false to hold its outputs neutral
     */
    public void requestMechEnabled(MwSubsystemBase subsystem, SwitchableIo mech, boolean enabled) {
        if (!switchFor(subsystem).requestMech(mech, enabled)) {
            throw new IllegalArgumentException(mech.getIoName() + " is not an IO of " + subsystem.getName());
        }
    }

    /**
     * @param subsystem a registered subsystem
     * @return its current mode
     */
    public SubsystemMode getMode(MwSubsystemBase subsystem) {
        return switchFor(subsystem).mode();
    }

    /**
     * @param subsystem a registered subsystem
     * @return true if it is {@link SubsystemMode#ENABLED}
     */
    public boolean isAvailable(MwSubsystemBase subsystem) {
        return getMode(subsystem) == SubsystemMode.ENABLED;
    }

    /**
     * @param subsystem the registered subsystem that owns the mechanism
     * @param mech the mechanism
     * @return false while the mechanism is switched off
     */
    public boolean isMechEnabled(MwSubsystemBase subsystem, SwitchableIo mech) {
        return switchFor(subsystem).isMechEnabled(mech);
    }

    /**
     * @param subsystem a registered subsystem
     * @param mode the mode to watch
     * @return a trigger that is true while the subsystem is in {@code mode}
     */
    public Trigger inMode(MwSubsystemBase subsystem, SubsystemMode mode) {
        SubsystemSwitch sw = switchFor(subsystem);
        return new Trigger(() -> sw.mode() == mode);
    }

    /**
     * @param name a subsystem's name (its class name without {@code Subsystem})
     * @return the registered subsystem with that name, or null
     */
    public MwSubsystemBase getSubsystem(String name) {
        for (MwSubsystemBase s : subsystems) {
            if (s.getName().equals(name)) {
                return s;
            }
        }
        return null;
    }

    /**
     * Serves the subsystem console, a control page bundled in MW-Lib, from the robot. Browse to
     * {@code http://<robot>:<port>/}.
     *
     * @param port the HTTP port, e.g. {@link SubsystemConsole#DEFAULT_PORT}
     */
    public void startSubsystemConsole(int port) {
        SubsystemConsole.start(port);
    }

    /** Preform the control loop for all subsystems */
    public void doControlLoop() {
        // Poll tunables and fire onChange consumers before subsystem logic runs
        MwLog.periodic();

        // Republish field region outlines if the alliance changed
        RegionPublisher.periodic();

        // Apply mode switches made since the last loop (usually none)
        for (Runnable event; (event = events_.poll()) != null; ) {
            try {
                event.run();
            } catch (Exception e) {
                DataLogManager.log(" Failed to apply a subsystem mode change");
                e.printStackTrace();
            }
        }

        // Let a coordinator (see MwSuperStateMachine) request subsystem states first, so its
        // requests take effect in this same loop
        update(MwLog.timestampSeconds());

        for (MwSubsystemBase subsystem : running_) {
            runSubsystem(subsystem, true);
        }
        for (MwSubsystemBase subsystem : parked_) {
            runSubsystem(subsystem, false);
        }
        // Log battery data every loop
        if (start_logging_) {
            BatteryLogger.logBatteryData();
        }
    }

    /**
     * One subsystem's loop: read inputs, run its logic if it is ENABLED, write outputs, log.
     *
     * @param subsystem the subsystem
     * @param run_logic false for DISABLED and TUNING subsystems
     */
    private void runSubsystem(MwSubsystemBase subsystem, boolean run_logic) {
        try {
            MwLog.time(subsystem.getSubsystemKey() + "/loop_time");

            List<SubsystemIoBase> ios = subsystem.getIos();

            // Deterministic timestamp — sourced from the log during replay
            double timestamp = MwLog.timestampSeconds();

            for (SubsystemIoBase io : ios) {
                io.readInputs(timestamp);
            }

            if (run_logic) {
                subsystem.update(timestamp);
            }

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
        // Nothing stays in TUNING into a match (applies at the top of the next loop)
        if (DriverStation.isFMSAttached()) {
            for (MwSubsystemBase subsystem : subsystems) {
                if (switches_.get(subsystem).mode() == SubsystemMode.TUNING) {
                    requestMode(subsystem, SubsystemMode.ENABLED);
                }
            }
        }
    }
}
