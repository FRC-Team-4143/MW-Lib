package com.marswars.subsystem;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.marswars.logging.BatteryLogger;
import com.marswars.logging.MwLog;
import com.marswars.regions.RegionPublisher;
import com.marswars.util.RobotIdentity;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Runs every registered subsystem's control loop.
 *
 * <p>Every registered subsystem gets a live {@link SubsystemControl} switch on NetworkTables at
 * {@code /MwLib/Subsystems/<Name>/Control}: {@code ROBOT} (the default, follows wanted-state
 * requests), {@code PINNED} (held in the state at {@code /MwLib/Subsystems/<Name>/PinnedState}) or
 * {@code OFF} (logic skipped, outputs neutral, inputs still logged). Changes apply at the top of the
 * next loop, are recorded as AdvantageKit inputs for replay, and raise an alert while a subsystem is
 * not under robot control. With the FMS attached, {@code PINNED} is ignored; {@code OFF} is honoured
 * unless {@link #setAllowOffWhenFmsAttached(boolean)} turns it off.
 */
public abstract class SubsystemManager {
    protected ArrayList<MwSubsystemBase> subsystems;
    protected Notifier loopThread;
    protected boolean log_init = false;
    private final boolean start_logging_;

    private static StringPublisher robot_name_pub_ =
            NetworkTableInstance.getDefault().getStringTopic("/Metadata/ROBOT_NAME").publish();

    protected static List<String> disabled_subsystems_;

    private final Map<MwSubsystemBase, SubsystemControlTable> control_tables_ = new HashMap<>();
    private boolean allow_off_with_fms_ = true;

    /**
     * @return the names of the subsystems that were left out at build time (the constructor's
     *     {@code disabled_subsystems} list)
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
            control_tables_.put(system, new SubsystemControlTable(system));
        }
    }

    /**
     * Asks for a subsystem's control from code, the same as a dashboard write: it takes effect at
     * the top of the next loop and dashboards show it.
     *
     * @param subsystem a registered subsystem
     * @param control the control to request; use {@link #requestPinnedState} to pin
     */
    public void requestControl(MwSubsystemBase subsystem, SubsystemControl control) {
        tableFor(subsystem).request(control, null);
    }

    /**
     * Pins a subsystem to a state from code (see {@link SubsystemControl#PINNED}).
     *
     * @param subsystem a registered subsystem
     * @param state the state to hold it in
     */
    public <S extends Enum<S>> void requestPinnedState(MwSubsystem<S, ?> subsystem, S state) {
        tableFor(subsystem).request(SubsystemControl.PINNED, state.name());
    }

    /**
     * @param subsystem a registered subsystem
     * @param control the control to request
     * @return a command that requests {@code control} once; it also runs while disabled
     */
    public Command requestControlCommand(MwSubsystemBase subsystem, SubsystemControl control) {
        return Commands.runOnce(() -> requestControl(subsystem, control))
                .ignoringDisable(true)
                .withName(subsystem.getName() + " control " + control);
    }

    /**
     * @param subsystem a registered subsystem
     * @return the control applied on the last loop
     */
    public SubsystemControl getControl(MwSubsystemBase subsystem) {
        return tableFor(subsystem).getActive();
    }

    /**
     * @param subsystem a registered subsystem
     * @return true if the subsystem follows wanted-state requests (its control is {@link
     *     SubsystemControl#ROBOT}); check it only where the robot should choose a different plan,
     *     requests to an unavailable subsystem are already dropped
     */
    public boolean isAvailable(MwSubsystemBase subsystem) {
        return getControl(subsystem) == SubsystemControl.ROBOT;
    }

    /**
     * @param subsystem a registered subsystem
     * @param control the control to watch
     * @return a trigger that is true while the subsystem's control is {@code control}
     */
    public Trigger inControl(MwSubsystemBase subsystem, SubsystemControl control) {
        SubsystemControlTable table = tableFor(subsystem);
        return new Trigger(() -> table.getActive() == control);
    }

    /**
     * Whether {@link SubsystemControl#OFF} is honoured while the FMS is attached, so a broken
     * mechanism can be shut off mid-match. Default true. {@link SubsystemControl#PINNED} is always
     * ignored with the FMS attached.
     *
     * @param allow true to honour OFF during matches
     */
    protected void setAllowOffWhenFmsAttached(boolean allow) {
        allow_off_with_fms_ = allow;
    }

    private SubsystemControlTable tableFor(MwSubsystemBase subsystem) {
        SubsystemControlTable table = control_tables_.get(Objects.requireNonNull(subsystem));
        if (table == null) {
            throw new IllegalArgumentException(subsystem.getName() + " is not registered");
        }
        return table;
    }

    /** Preform the control loop for all subsystems */
    public void doControlLoop() {
        // Poll tunables and fire onChange consumers before subsystem logic runs
        MwLog.periodic();

        // Republish field region outlines if the alliance changed
        RegionPublisher.periodic();

        // Apply dashboard control changes before anyone requests states this loop
        boolean fms_attached = DriverStation.isFMSAttached();
        for (MwSubsystemBase subsystem : subsystems) {
            try {
                control_tables_.get(subsystem).periodic(fms_attached, allow_off_with_fms_);
            } catch (Exception e) {
                DataLogManager.log(
                        " Failed to apply control for " + subsystem.getClass().getCanonicalName());
                e.printStackTrace();
            }
        }

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

                boolean off = control_tables_.get(subsystem).getActive() == SubsystemControl.OFF;

                for (SubsystemIoBase io : ios) {
                    io.readInputs(timestamp);
                    io.setNeutralOutputs(off);
                }

                // An OFF subsystem's logic does not run; its IO still writes (neutral) and logs
                if (!off) {
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
