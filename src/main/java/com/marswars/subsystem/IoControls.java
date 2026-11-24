package com.marswars.subsystem;

import com.marswars.logging.MwLog;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * One IO's live controls, attached by {@link SubsystemManager} to every {@link ControllableIo} in a
 * registered subsystem. Entries live at {@code /MwLib/Subsystems/<Sub>/IO/<Name>/}:
 *
 * <ul>
 *   <li>{@code Enabled}: false switches the IO off. A mechanism holds its outputs neutral (its
 *       subsystem keeps running, and the mechanism follows it again when switched back on); a
 *       sensor reports disconnected and returns a safe default. Honoured with the FMS attached
 *       unless {@link SubsystemManager#setAllowOffWhenFmsAttached(boolean)} says otherwise.
 *   <li>{@code Override/<Channel>/Active} and {@code Override/<Channel>/Value}: replace one sensor
 *       reading (e.g. {@code Position}, {@code Range}, {@code Yaw}) in the IO's getters. The raw
 *       reading is still logged. Ignored while the FMS is attached.
 *   <li>{@code Manual/Mode}, {@code Manual/Setpoint}: the output while the subsystem is {@link
 *       SubsystemControl#MANUAL}. {@code Manual/Modes} lists the modes the IO supports.
 *   <li>{@code Manual/Step}, {@code Manual/StepTo}, {@code Manual/StepPeriod}: a step test that
 *       alternates the setpoint between {@code Setpoint} and {@code StepTo}, each held for {@code
 *       StepPeriod} seconds.
 * </ul>
 *
 * Every entry is read as an AdvantageKit input, so replay reproduces it.
 */
public final class IoControls {
    /** One dashboard-written entry, read each loop and logged as an input. */
    private abstract static class Field {
        final String key;

        Field(String key) {
            this.key = key;
        }

        abstract void read();

        abstract void toLog(LogTable table);

        abstract void fromLog(LogTable table);
    }

    private static final class BoolField extends Field {
        final BooleanEntry entry;
        boolean value;

        BoolField(NetworkTable table, String key, boolean initial) {
            super(key);
            entry = table.getBooleanTopic(key).getEntry(initial);
            entry.set(initial);
            value = initial;
        }

        void read() {
            value = entry.get();
        }

        void write(boolean v) {
            entry.set(v);
            value = v;
        }

        void toLog(LogTable t) {
            t.put(key, value);
        }

        void fromLog(LogTable t) {
            value = t.get(key, value);
        }
    }

    private static final class DoubleField extends Field {
        final DoubleEntry entry;
        double value;

        DoubleField(NetworkTable table, String key, double initial) {
            super(key);
            entry = table.getDoubleTopic(key).getEntry(initial);
            entry.set(initial);
            value = initial;
        }

        void read() {
            value = entry.get();
        }

        void write(double v) {
            entry.set(v);
            value = v;
        }

        void toLog(LogTable t) {
            t.put(key, value);
        }

        void fromLog(LogTable t) {
            value = t.get(key, value);
        }
    }

    private static final class StringField extends Field {
        final StringEntry entry;
        String value;

        StringField(NetworkTable table, String key, String initial) {
            super(key);
            entry = table.getStringTopic(key).getEntry(initial);
            entry.set(initial);
            value = initial;
        }

        void read() {
            value = entry.get();
        }

        void write(String v) {
            entry.set(v);
            value = v;
        }

        void toLog(LogTable t) {
            t.put(key, value);
        }

        void fromLog(LogTable t) {
            value = t.get(key, value);
        }
    }

    private record Sensor(BoolField active, DoubleField value) {}

    private final ControllableIo io_;
    private final String display_name_;
    private final String log_key_;
    private final NetworkTable table_;
    private final List<Field> fields_ = new ArrayList<>();
    private final Map<String, Sensor> sensors_ = new LinkedHashMap<>();
    private final List<ManualMode> manual_modes_;

    private final BoolField enabled_;
    private final StringField manual_mode_;
    private final DoubleField manual_setpoint_;
    private final BoolField step_;
    private final DoubleField step_to_;
    private final DoubleField step_period_;

    private final Alert status_alert_;
    private final Alert problem_alert_;

    private final LoggableInputs inputs_ =
            new LoggableInputs() {
                @Override
                public void toLog(LogTable table) {
                    for (Field f : fields_) {
                        f.toLog(table);
                    }
                }

                @Override
                public void fromLog(LogTable table) {
                    for (Field f : fields_) {
                        f.fromLog(table);
                    }
                }
            };

    // decided each loop
    private boolean enabled_now_ = true;
    private boolean overrides_allowed_ = true;
    private boolean manual_ = false;
    private ManualMode mode_now_ = ManualMode.NEUTRAL;
    private double setpoint_now_ = 0.0;

    /**
     * @param io the IO these controls belong to
     * @param subsystem_name the owning subsystem's name
     * @param io_name the IO's name, unique within the subsystem
     */
    IoControls(ControllableIo io, String subsystem_name, String io_name) {
        io_ = io;
        display_name_ = subsystem_name + "/" + io_name;
        log_key_ = "MwLib/Subsystems/" + subsystem_name + "/IO/" + io_name;
        table_ =
                NetworkTableInstance.getDefault()
                        .getTable(SubsystemControlTable.NT_ROOT + subsystem_name + "/IO/" + io_name);

        enabled_ = add(new BoolField(table_, "Enabled", true));

        manual_modes_ = List.copyOf(io.getManualModes());
        if (manual_modes_.isEmpty()) {
            manual_mode_ = null;
            manual_setpoint_ = null;
            step_ = null;
            step_to_ = null;
            step_period_ = null;
        } else {
            table_.getStringArrayTopic("Manual/Modes")
                    .publish()
                    .set(manual_modes_.stream().map(Enum::name).toArray(String[]::new));
            manual_mode_ = add(new StringField(table_, "Manual/Mode", ManualMode.NEUTRAL.name()));
            manual_setpoint_ = add(new DoubleField(table_, "Manual/Setpoint", 0.0));
            step_ = add(new BoolField(table_, "Manual/Step", false));
            step_to_ = add(new DoubleField(table_, "Manual/StepTo", 0.0));
            step_period_ = add(new DoubleField(table_, "Manual/StepPeriod", 2.0));
        }

        status_alert_ = new Alert("", AlertType.kWarning);
        problem_alert_ = new Alert("", AlertType.kError);
    }

    private <F extends Field> F add(F field) {
        fields_.add(field);
        return field;
    }

    /**
     * Declares a sensor reading that can be overridden from the dashboard. Call from {@link
     * ControllableIo#attachControls(IoControls)}.
     *
     * @param channel the reading's name, e.g. {@code Position}
     */
    public void declareSensor(String channel) {
        if (sensors_.containsKey(channel)) {
            return;
        }
        BoolField active = add(new BoolField(table_, "Override/" + channel + "/Active", false));
        DoubleField value = add(new DoubleField(table_, "Override/" + channel + "/Value", 0.0));
        sensors_.put(channel, new Sensor(active, value));
    }

    /**
     * @param channel a channel from {@link #declareSensor(String)}
     * @param raw the measured value
     * @return the override if one is active (and the FMS is not attached), otherwise {@code raw}
     */
    public double sensorValue(String channel, double raw) {
        return isSensorOverridden(channel) ? sensors_.get(channel).value().value : raw;
    }

    /**
     * @param channel a channel from {@link #declareSensor(String)}
     * @return true if the channel's reading is being replaced this loop
     */
    public boolean isSensorOverridden(String channel) {
        Sensor sensor = sensors_.get(channel);
        return sensor != null && overrides_allowed_ && sensor.active().value;
    }

    /** @return false while the IO is switched off */
    public boolean isEnabled() {
        return enabled_now_;
    }

    /** @return true while the IO's own controls hold its outputs neutral (switched off, or manual NEUTRAL) */
    public boolean holdsNeutral() {
        return !enabled_now_ || (manual_ && mode_now_ == ManualMode.NEUTRAL);
    }

    /** @return the manual mode applied this loop, NEUTRAL when not in manual */
    public ManualMode getManualMode() {
        return manual_ ? mode_now_ : ManualMode.NEUTRAL;
    }

    /** @return the manual setpoint applied this loop (step test and clamps included) */
    public double getManualSetpoint() {
        return setpoint_now_;
    }

    /**
     * Reads the entries and decides this loop's switch, overrides and manual output. Called by the
     * manager at the top of each loop.
     */
    void periodic(
            SubsystemControl control,
            boolean fms_attached,
            boolean allow_off_with_fms,
            double timestamp,
            double duty_cycle_limit) {
        for (Field f : fields_) {
            f.read();
        }
        Logger.processInputs(log_key_, inputs_);

        enabled_now_ = enabled_.value || (fms_attached && !allow_off_with_fms);
        overrides_allowed_ = !fms_attached;

        boolean entering_manual = control == SubsystemControl.MANUAL && !manual_;
        manual_ = control == SubsystemControl.MANUAL;

        String problem = null;
        mode_now_ = ManualMode.NEUTRAL;
        setpoint_now_ = 0.0;
        if (manual_ && manual_mode_ != null) {
            if (entering_manual) {
                seedManual();
            }
            ManualMode requested = parseMode(manual_mode_.value);
            if (requested == null || !manual_modes_.contains(requested)) {
                problem =
                        display_name_ + ": manual mode \"" + manual_mode_.value
                                + "\" not supported, holding NEUTRAL";
                requested = ManualMode.NEUTRAL;
            }
            double setpoint = manual_setpoint_.value;
            double period = step_period_.value;
            if (step_.value && period > 0.0 && Math.floorMod((long) Math.floor(timestamp / period), 2L) == 1L) {
                setpoint = step_to_.value;
            }
            if (requested == ManualMode.DUTY_CYCLE) {
                setpoint = Math.max(-duty_cycle_limit, Math.min(duty_cycle_limit, setpoint));
            }
            mode_now_ = requested;
            setpoint_now_ = setpoint;
        }

        MwLog.log(log_key_ + "/EnabledApplied", enabled_now_);
        MwLog.log(log_key_ + "/ManualModeApplied", getManualMode().name());
        MwLog.log(log_key_ + "/ManualSetpointApplied", setpoint_now_);

        List<String> status = new ArrayList<>();
        if (!enabled_now_) {
            status.add("OFF");
        }
        if (manual_ && enabled_now_) {
            status.add("manual " + mode_now_ + (mode_now_ == ManualMode.NEUTRAL ? "" : " " + setpoint_now_));
        }
        for (Map.Entry<String, Sensor> e : sensors_.entrySet()) {
            if (isSensorOverridden(e.getKey())) {
                status.add(e.getKey() + " overridden to " + e.getValue().value().value);
            }
        }
        status_alert_.setText(status.isEmpty() ? "" : display_name_ + ": " + String.join(", ", status));
        status_alert_.set(!status.isEmpty());
        problem_alert_.setText(problem == null ? "" : problem);
        problem_alert_.set(problem != null);
    }

    /** Applies the manual output, if any, after inputs are read. Called by the manager. */
    void applyManual() {
        if (manual_ && enabled_now_ && mode_now_ != ManualMode.NEUTRAL) {
            io_.applyManualOutput(mode_now_, setpoint_now_);
        }
    }

    /** Position mechanisms hold where they are; velocity mechanisms start at zero. */
    private void seedManual() {
        ManualMode mode;
        if (manual_modes_.contains(ManualMode.POSITION)) {
            mode = ManualMode.POSITION;
        } else if (manual_modes_.contains(ManualMode.VELOCITY)) {
            mode = ManualMode.VELOCITY;
        } else {
            mode = ManualMode.NEUTRAL;
        }
        double seed = io_.getManualSeed(mode);
        manual_mode_.write(mode.name());
        manual_setpoint_.write(seed);
        step_.write(false);
        step_to_.write(seed);
    }

    private static ManualMode parseMode(String text) {
        if (text == null) {
            return null;
        }
        try {
            return ManualMode.valueOf(text.trim().toUpperCase(Locale.ROOT));
        } catch (IllegalArgumentException e) {
            return null;
        }
    }
}
