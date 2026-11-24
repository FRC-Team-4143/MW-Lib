package com.marswars.subsystem;

import com.marswars.logging.MwLog;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.Locale;

/**
 * One subsystem's live control entries on NetworkTables:
 *
 * <ul>
 *   <li>{@code /MwLib/Subsystems/<Name>/Control} (dashboard writes): {@code ROBOT}, {@code PINNED},
 *       {@code MANUAL} or {@code OFF}
 *   <li>{@code /MwLib/Subsystems/<Name>/PinnedState} (dashboard writes): a state name
 *   <li>{@code /MwLib/Subsystems/<Name>/ActiveControl} (robot writes): what was applied
 *   <li>{@code /MwLib/Subsystems/<Name>/States} (robot writes): the state names a dashboard can
 *       offer for pinning
 * </ul>
 *
 * The two dashboard entries are read as AdvantageKit inputs, so replay reproduces every change.
 */
final class SubsystemControlTable {
    static final String NT_ROOT = "/MwLib/Subsystems/";

    /** What the dashboard asked for, as logged inputs. */
    private static final class Inputs implements LoggableInputs {
        String control = SubsystemControl.ROBOT.name();
        String pinned_state = "";

        @Override
        public void toLog(LogTable table) {
            table.put("Control", control);
            table.put("PinnedState", pinned_state);
        }

        @Override
        public void fromLog(LogTable table) {
            control = table.get("Control", control);
            pinned_state = table.get("PinnedState", pinned_state);
        }
    }

    private final MwSubsystemBase subsystem_;
    private final String log_key_;
    private final StringEntry control_entry_;
    private final StringEntry pinned_entry_;
    private final StringPublisher active_pub_;
    private final StringArrayPublisher states_pub_;
    private final Inputs inputs_ = new Inputs();
    private final Alert status_alert_;
    private final Alert problem_alert_;

    private SubsystemControl active_ = SubsystemControl.ROBOT;

    SubsystemControlTable(MwSubsystemBase subsystem) {
        subsystem_ = subsystem;
        log_key_ = "MwLib/Subsystems/" + subsystem.getName();
        NetworkTable table =
                NetworkTableInstance.getDefault().getTable(NT_ROOT + subsystem.getName());

        // Every boot starts with the robot in control, whatever a dashboard last wrote
        control_entry_ = table.getStringTopic("Control").getEntry(SubsystemControl.ROBOT.name());
        control_entry_.set(SubsystemControl.ROBOT.name());
        pinned_entry_ = table.getStringTopic("PinnedState").getEntry("");
        pinned_entry_.set("");

        active_pub_ = table.getStringTopic("ActiveControl").publish();
        active_pub_.set(SubsystemControl.ROBOT.name());
        states_pub_ = table.getStringArrayTopic("States").publish();
        states_pub_.set(subsystem.getStateNames().toArray(new String[0]));

        status_alert_ = new Alert("", AlertType.kWarning);
        problem_alert_ = new Alert("", AlertType.kError);
    }

    /**
     * Reads the dashboard entries, decides the control for this loop and applies it to the
     * subsystem.
     *
     * @param fms_attached whether the FMS is attached this loop
     * @param allow_off_with_fms whether {@link SubsystemControl#OFF} is honoured with the FMS
     *     attached
     */
    void periodic(boolean fms_attached, boolean allow_off_with_fms) {
        inputs_.control = control_entry_.get();
        inputs_.pinned_state = pinned_entry_.get();
        Logger.processInputs(log_key_, inputs_);

        String name = subsystem_.getName();
        String problem = null;
        SubsystemControl requested = parse(inputs_.control);
        if (requested == null) {
            problem = name + ": unknown control \"" + inputs_.control + "\", using ROBOT";
            requested = SubsystemControl.ROBOT;
        } else if (requested == SubsystemControl.PINNED
                && !subsystem_.getStateNames().contains(inputs_.pinned_state)) {
            problem = name + ": can't pin to unknown state \"" + inputs_.pinned_state + "\", using ROBOT";
            requested = SubsystemControl.ROBOT;
        }

        SubsystemControl applied = requested;
        boolean fms_locked =
                fms_attached
                        && (requested == SubsystemControl.PINNED
                                || requested == SubsystemControl.MANUAL
                                || (requested == SubsystemControl.OFF && !allow_off_with_fms));
        if (fms_locked) {
            applied = SubsystemControl.ROBOT;
        }

        active_ = applied;
        subsystem_.applyControl(
                applied, applied == SubsystemControl.PINNED ? inputs_.pinned_state : null);

        active_pub_.set(applied.name());
        MwLog.log(subsystem_.getSubsystemKey() + "Control", applied);

        problem_alert_.setText(problem == null ? "" : problem);
        problem_alert_.set(problem != null);

        String status =
                switch (applied) {
                    case ROBOT -> fms_locked ? name + ": " + requested + " ignored while FMS attached" : null;
                    case PINNED -> name + " pinned to " + inputs_.pinned_state;
                    case MANUAL -> name + " in MANUAL";
                    case OFF -> name + " is OFF";
                };
        status_alert_.setText(status == null ? "" : status);
        status_alert_.set(status != null);
    }

    /** @return the control applied on the last {@link #periodic} */
    SubsystemControl getActive() {
        return active_;
    }

    /** Writes the dashboard entries from code, so dashboards show the request. */
    void request(SubsystemControl control, String pinned_state) {
        if (pinned_state != null) {
            pinned_entry_.set(pinned_state);
        }
        control_entry_.set(control.name());
    }

    private static SubsystemControl parse(String text) {
        if (text == null) {
            return null;
        }
        try {
            return SubsystemControl.valueOf(text.trim().toUpperCase(Locale.ROOT));
        } catch (IllegalArgumentException e) {
            return null;
        }
    }
}
