package com.marswars.subsystem;

import com.marswars.logging.MwLog;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Queue;
import java.util.Set;

/**
 * One subsystem's live switches on NetworkTables:
 *
 * <ul>
 *   <li>{@code /MwLib/Subsystems/<Name>/Mode}: {@code ENABLED}, {@code DISABLED} or {@code TUNING}
 *   <li>{@code /MwLib/Subsystems/<Name>/ActiveMode}: what the robot applied
 *   <li>{@code /MwLib/Subsystems/<Name>/Mechs/<Mech>/Enabled}: one mechanism's switch
 *   <li>{@code /MwLib/Subsystems/<Name>/Mechs/<Mech>/LoggingKey}: where its tunables and inputs are
 * </ul>
 *
 * Nothing here runs per loop. A write to a switch fires an NT listener, which queues the change;
 * {@link SubsystemManager} applies queued changes at the top of its next loop, on the robot thread.
 */
final class SubsystemSwitch {
    static final String NT_ROOT = "/MwLib/Subsystems/";

    private final class MechSwitch {
        final SwitchableIo io;
        final String name;
        final BooleanEntry enabled_entry;
        final Alert alert;
        boolean enabled = true;

        MechSwitch(SwitchableIo io, String name, NetworkTable table) {
            this.io = io;
            this.name = name;
            NetworkTable mech = table.getSubTable("Mechs").getSubTable(name);
            enabled_entry = mech.getBooleanTopic("Enabled").getEntry(true);
            enabled_entry.set(true);
            StringPublisher key = mech.getStringTopic("LoggingKey").publish();
            key.set(io.getLoggingKey() == null ? "" : io.getLoggingKey());
            keep_.add(key);
            alert = new Alert(subsystem.getName() + "/" + name + " is switched off", AlertType.kWarning);
            NetworkTableInstance.getDefault()
                    .addListener(
                            enabled_entry,
                            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                            e -> events_.add(() -> setMechEnabled(this, e.valueData.value.getBoolean())));
        }
    }

    final MwSubsystemBase subsystem;
    private final Queue<Runnable> events_;
    private final Runnable on_mode_change_;
    private final StringEntry mode_entry_;
    private final StringPublisher active_pub_;
    private final Alert alert_;
    private final List<MechSwitch> mechs_ = new ArrayList<>();
    private final List<Object> keep_ = new ArrayList<>();
    private SubsystemMode mode_ = SubsystemMode.ENABLED;

    /**
     * @param subsystem the subsystem
     * @param events the manager's queue, drained on the robot thread
     * @param on_mode_change called (on the robot thread) after the mode changed
     */
    SubsystemSwitch(MwSubsystemBase subsystem, Queue<Runnable> events, Runnable on_mode_change) {
        this.subsystem = subsystem;
        events_ = events;
        on_mode_change_ = on_mode_change;
        NetworkTable table = NetworkTableInstance.getDefault().getTable(NT_ROOT + subsystem.getName());

        // Every boot starts enabled, whatever a dashboard last wrote
        mode_entry_ = table.getStringTopic("Mode").getEntry(SubsystemMode.ENABLED.name());
        mode_entry_.set(SubsystemMode.ENABLED.name());
        active_pub_ = table.getStringTopic("ActiveMode").publish();
        active_pub_.set(SubsystemMode.ENABLED.name());
        alert_ = new Alert("", AlertType.kWarning);

        Set<String> names = new HashSet<>();
        List<SubsystemIoBase> ios = subsystem.getIos();
        if (ios != null) {
            for (SubsystemIoBase io : ios) {
                if (io instanceof SwitchableIo sw) {
                    String name = sw.getIoName();
                    for (int i = 2; names.contains(name); i++) {
                        name = sw.getIoName() + i;
                    }
                    names.add(name);
                    mechs_.add(new MechSwitch(sw, name, table));
                }
            }
        }

        NetworkTableInstance.getDefault()
                .addListener(
                        mode_entry_,
                        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                        e -> events_.add(() -> setMode(e.valueData.value.getString())));
    }

    SubsystemMode mode() {
        return mode_;
    }

    /** Asks for a mode; same as a dashboard write, applied at the top of the next loop. */
    void request(SubsystemMode mode) {
        mode_entry_.set(mode.name());
    }

    /** Asks for a mechanism's switch; applied at the top of the next loop. */
    boolean requestMech(SwitchableIo io, boolean enabled) {
        for (MechSwitch m : mechs_) {
            if (m.io == io) {
                m.enabled_entry.set(enabled);
                return true;
            }
        }
        return false;
    }

    /** @return false if {@code io} is switched off; true if on or not one of this subsystem's mechs */
    boolean isMechEnabled(SwitchableIo io) {
        for (MechSwitch m : mechs_) {
            if (m.io == io) {
                return m.enabled;
            }
        }
        return true;
    }

    /** Robot thread. */
    private void setMode(String text) {
        SubsystemMode mode;
        try {
            mode = SubsystemMode.valueOf(text.trim().toUpperCase(Locale.ROOT));
        } catch (IllegalArgumentException | NullPointerException e) {
            DataLogManager.log(subsystem.getName() + ": unknown mode \"" + text + "\"");
            mode_entry_.set(mode_.name());
            return;
        }
        if (mode == mode_) {
            return;
        }
        if (mode == SubsystemMode.TUNING && DriverStation.isFMSAttached()) {
            DataLogManager.log(subsystem.getName() + ": TUNING refused while the FMS is attached");
            mode_entry_.set(mode_.name());
            return;
        }

        mode_ = mode;
        for (SubsystemIoBase io : subsystem.getIos()) {
            io.setNeutralOutputs(mode == SubsystemMode.DISABLED || !isIoOn(io));
        }
        if (mode == SubsystemMode.TUNING) {
            for (MechSwitch m : mechs_) {
                m.io.hold();
            }
        } else if (mode == SubsystemMode.ENABLED) {
            subsystem.onEnable();
        }

        active_pub_.set(mode.name());
        MwLog.log(subsystem.getSubsystemKey() + "Mode", mode);
        DataLogManager.log(subsystem.getName() + " mode " + mode);
        alert_.setText(subsystem.getName() + " is " + mode);
        alert_.set(mode != SubsystemMode.ENABLED);
        on_mode_change_.run();
    }

    private boolean isIoOn(SubsystemIoBase io) {
        return !(io instanceof SwitchableIo sw) || isMechEnabled(sw);
    }

    /** Robot thread. */
    private void setMechEnabled(MechSwitch m, boolean enabled) {
        if (m.enabled == enabled) {
            return;
        }
        m.enabled = enabled;
        m.io.setNeutralOutputs(!enabled || mode_ == SubsystemMode.DISABLED);
        if (enabled && mode_ == SubsystemMode.TUNING) {
            m.io.hold(); // don't jump to a stale target
        }
        MwLog.log(subsystem.getSubsystemKey() + "Mechs/" + m.name + "/Enabled", enabled);
        DataLogManager.log(subsystem.getName() + "/" + m.name + (enabled ? " on" : " off"));
        m.alert.set(!enabled);
    }
}
