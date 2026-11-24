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
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * Runs every registered subsystem's control loop.
 *
 * <p>Every registered subsystem gets a live {@link SubsystemControl} switch on NetworkTables at
 * {@code /MwLib/Subsystems/<Name>/Control}: {@code ROBOT} (the default, follows wanted-state
 * requests), {@code PINNED} (held in the state at {@code /MwLib/Subsystems/<Name>/PinnedState}) or
 * {@code MANUAL} (logic skipped, mechanisms driven by hand) or {@code OFF} (logic skipped, outputs
 * neutral, inputs still logged). Changes apply at the top of the next loop, are recorded as
 * AdvantageKit inputs for replay, and raise an alert while a subsystem is not under robot control.
 *
 * <p>Every library IO in a registered subsystem ({@link ControllableIo}: each {@code *Mech}, {@code
 * Tof}, {@code Gyro}) also gets its own on/off switch, sensor overrides and, for mechanisms, a
 * manual output, at {@code /MwLib/Subsystems/<Name>/IO/<IoName>/} (see {@link IoControls}).
 *
 * <p>A library IO's tunables (keys under its logging key, e.g. motor gains) only accept changes
 * while its subsystem is in {@code MANUAL}; {@link #setTuneOnlyInManual(boolean)} turns this off.
 * Other tunables stay live. No tunable changes while the FMS is attached. Leaving {@code MANUAL}
 * logs every value of the subsystem's that changed, to copy into constants.
 *
 * <p>With the FMS attached, {@code PINNED}, {@code MANUAL} and sensor overrides are ignored; {@code
 * OFF} (subsystem or IO) is honoured unless {@link #setAllowOffWhenFmsAttached(boolean)} turns it
 * off.
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
    private double manual_duty_cycle_limit_ = 0.30;
    private boolean tune_only_in_manual_ = true;

    private final Map<MwSubsystemBase, List<IoControls>> io_controls_ = new HashMap<>();
    private final Map<ControllableIo, IoControls> controls_by_io_ = new IdentityHashMap<>();
    // every tunable a subsystem owns (for the tuned-values report), and just its IOs' (gated)
    private final Map<MwSubsystemBase, List<String>> tunable_prefixes_ = new LinkedHashMap<>();
    private final Map<MwSubsystemBase, List<String>> io_tunable_prefixes_ = new LinkedHashMap<>();
    private final Map<MwSubsystemBase, Map<String, Double>> tuning_start_values_ = new HashMap<>();

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

        MwLog.setTunableGate(this::acceptsTunableChange);
    }

    public void registerSubsystem(MwSubsystemBase system) {
        if (disabled_subsystems_.contains(system.getName())) {
            DataLogManager.log(
                    "Registered disabled subsystem: " + system.getClass().getSimpleName());
        } else {
            subsystems.add(system);
            control_tables_.put(system, new SubsystemControlTable(system));
            io_controls_.put(system, new ArrayList<>());
            List<String> prefixes = new ArrayList<>();
            prefixes.add(system.getSubsystemKey());
            tunable_prefixes_.put(system, prefixes);
            io_tunable_prefixes_.put(system, new ArrayList<>());
            attachIos(system);
        }
    }

    /** Attaches controls to every library IO (and child IO) the subsystem has not had attached. */
    private void attachIos(MwSubsystemBase system) {
        List<SubsystemIoBase> ios = system.getIos();
        if (ios == null) {
            return;
        }
        for (SubsystemIoBase io : ios) {
            if (io instanceof ControllableIo controllable) {
                attachIo(system, controllable);
            }
        }
    }

    private void attachIo(MwSubsystemBase system, ControllableIo io) {
        if (controls_by_io_.containsKey(io)) {
            return;
        }
        List<IoControls> list = io_controls_.get(system);
        Set<String> taken = new HashSet<>();
        for (Map.Entry<ControllableIo, IoControls> e : controls_by_io_.entrySet()) {
            if (list.contains(e.getValue())) {
                taken.add(e.getKey().getIoName());
            }
        }
        String name = io.getIoName();
        for (int i = 2; taken.contains(name); i++) {
            name = io.getIoName() + i;
        }
        IoControls controls = new IoControls(io, system.getName(), name);
        controls_by_io_.put(io, controls);
        list.add(controls);
        io.attachControls(controls);

        String key = io.getLoggingKey();
        if (key != null) {
            tunable_prefixes_.get(system).add(key);
            io_tunable_prefixes_.get(system).add(key);
        }
        for (ControllableIo child : io.getChildIos()) {
            attachIo(system, child);
        }
    }

    /**
     * @param io an IO of a registered subsystem
     * @return its controls, or null if it is not a {@link ControllableIo} of a registered subsystem
     */
    public IoControls getIoControls(ControllableIo io) {
        return controls_by_io_.get(io);
    }

    /**
     * The tunable gate. Nothing changes while the FMS is attached. A mechanism's or sensor's
     * tunables (keys under its logging key, e.g. motor gains) only change while its subsystem is in
     * MANUAL; other tunables, including subsystem-level gains that need the subsystem's logic
     * running, stay live.
     */
    boolean acceptsTunableChange(String key) {
        if (DriverStation.isFMSAttached()) {
            return false;
        }
        if (!tune_only_in_manual_) {
            return true;
        }
        for (Map.Entry<MwSubsystemBase, List<String>> e : io_tunable_prefixes_.entrySet()) {
            for (String prefix : e.getValue()) {
                if (key.startsWith(prefix)) {
                    return control_tables_.get(e.getKey()).getActive() == SubsystemControl.MANUAL;
                }
            }
        }
        return true;
    }

    private boolean ownsTunable(MwSubsystemBase subsystem, String key) {
        return tunable_prefixes_.get(subsystem).stream().anyMatch(key::startsWith);
    }

    /** On entering MANUAL, remember the tunables; on leaving, log the ones that changed. */
    private void trackTuning(MwSubsystemBase subsystem, SubsystemControl before, SubsystemControl after) {
        if (before != SubsystemControl.MANUAL && after == SubsystemControl.MANUAL) {
            tuning_start_values_.put(subsystem, MwLog.tunableValues(k -> ownsTunable(subsystem, k)));
        } else if (before == SubsystemControl.MANUAL && after != SubsystemControl.MANUAL) {
            Map<String, Double> start = tuning_start_values_.remove(subsystem);
            if (start == null) {
                return;
            }
            List<String> changes = new ArrayList<>();
            MwLog.tunableValues(k -> ownsTunable(subsystem, k))
                    .forEach(
                            (key, value) -> {
                                Double was = start.get(key);
                                if (was != null && !was.equals(value)) {
                                    changes.add(key + ": " + was + " -> " + value);
                                }
                            });
            MwLog.log(subsystem.getSubsystemKey() + "TunedValues", changes.toArray(new String[0]));
            if (!changes.isEmpty()) {
                DataLogManager.log(
                        subsystem.getName()
                                + " left MANUAL with tuned values (copy into constants):\n  "
                                + String.join("\n  ", changes));
            }
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
     * Whether {@link SubsystemControl#OFF}, and switching off a single IO, are honoured while the
     * FMS is attached, so a broken mechanism can be shut off mid-match. Default true. {@link
     * SubsystemControl#PINNED}, {@link SubsystemControl#MANUAL} and sensor overrides are always
     * ignored with the FMS attached.
     *
     * @param allow true to honour OFF during matches
     */
    protected void setAllowOffWhenFmsAttached(boolean allow) {
        allow_off_with_fms_ = allow;
    }

    /**
     * Largest duty cycle a mechanism can be driven at in {@link ManualMode#DUTY_CYCLE}. Default
     * 0.30.
     *
     * @param limit the limit, 0 to 1
     */
    protected void setManualDutyCycleLimit(double limit) {
        manual_duty_cycle_limit_ = Math.max(0.0, Math.min(1.0, limit));
    }

    /**
     * Whether a library IO's tunables (e.g. motor gains) only accept changes while its subsystem is
     * in {@link SubsystemControl#MANUAL}. Default true. When false, they change live in every
     * control, as before. Tunables never change while the FMS is attached.
     *
     * @param only true to gate tunables on MANUAL
     */
    protected void setTuneOnlyInManual(boolean only) {
        tune_only_in_manual_ = only;
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
        double now = MwLog.timestampSeconds();
        for (MwSubsystemBase subsystem : subsystems) {
            try {
                SubsystemControlTable table = control_tables_.get(subsystem);
                SubsystemControl before = table.getActive();
                table.periodic(fms_attached, allow_off_with_fms_);
                SubsystemControl after = table.getActive();
                trackTuning(subsystem, before, after);

                attachIos(subsystem);
                for (IoControls controls : io_controls_.get(subsystem)) {
                    controls.periodic(
                            after, fms_attached, allow_off_with_fms_, now, manual_duty_cycle_limit_);
                }
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

                SubsystemControl control = control_tables_.get(subsystem).getActive();
                boolean off = control == SubsystemControl.OFF;

                for (SubsystemIoBase io : ios) {
                    io.readInputs(timestamp);
                    io.setNeutralOutputs(off);
                }

                // OFF and MANUAL subsystems' logic does not run; their IO still writes and logs
                if (control == SubsystemControl.MANUAL) {
                    for (IoControls controls : io_controls_.get(subsystem)) {
                        controls.applyManual();
                    }
                } else if (!off) {
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
