package com.marswars.bt.monitor;

import com.marswars.bt.core.ParameterStore;
import com.marswars.bt.core.PortInfo;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/**
 * Tree parameters as dashboard-editable NetworkTables entries under {@code /Tuning/<key>} (the same
 * place as {@code MwLog.tunable}). Numbers ({@code int}, {@code double}) become number entries,
 * {@code bool} a boolean entry, anything else a string entry. They are AdvantageKit network inputs,
 * so edits are logged and replay identically. Values do not persist across reboots: copy settled
 * values back into the XML default.
 *
 * <p>When a reloaded XML changes a parameter's default, the entry follows the new default unless
 * someone has tuned it (its value differs from the old default).
 */
public final class NetworkParameterStore implements ParameterStore {
    public static final String PREFIX = "/Tuning/";

    private record Entry(Object input, String defaultValue) {}

    private final Map<String, Entry> entries_ = new HashMap<>();

    @Override
    public synchronized Object value(String key, PortInfo port) {
        String def = port.defaultValue() == null ? "" : port.defaultValue();
        Entry entry = entries_.get(key);
        if (entry == null) {
            entry = new Entry(create(PREFIX + key, port, def), def);
            entries_.put(key, entry);
        } else if (!entry.defaultValue().equals(def)) {
            // XML default edited and reloaded: follow it unless the value was tuned.
            followDefault(PREFIX + key, port, entry.defaultValue(), def);
            setDefault(entry.input(), port, def);
            entry = new Entry(entry.input(), def);
            entries_.put(key, entry);
        }
        return read(entry.input(), port);
    }

    private static Object create(String path, PortInfo port, String def) {
        return switch (port.type()) {
            case INT, DOUBLE -> new LoggedNetworkNumber(path, number(port, def));
            case BOOLEAN -> new LoggedNetworkBoolean(path, Boolean.TRUE.equals(ParameterStore.typed(port, def)));
            default -> new LoggedNetworkString(path, def);
        };
    }

    private static void followDefault(String path, PortInfo port, String oldDef, String def) {
        var entry = edu.wpi.first.networktables.NetworkTableInstance.getDefault().getEntry(path);
        switch (port.type()) {
            case INT, DOUBLE -> {
                if (entry.getDouble(Double.NaN) == number(port, oldDef)) {
                    entry.setDouble(number(port, def));
                }
            }
            case BOOLEAN -> {
                boolean old = Boolean.TRUE.equals(ParameterStore.typed(port, oldDef));
                if (entry.getBoolean(!old) == old) {
                    entry.setBoolean(Boolean.TRUE.equals(ParameterStore.typed(port, def)));
                }
            }
            default -> {
                if (oldDef.equals(entry.getString(null))) {
                    entry.setString(def);
                }
            }
        }
    }

    private static void setDefault(Object input, PortInfo port, String def) {
        if (input instanceof LoggedNetworkNumber n) {
            n.setDefault(number(port, def));
        } else if (input instanceof LoggedNetworkBoolean b) {
            b.setDefault(Boolean.TRUE.equals(ParameterStore.typed(port, def)));
        } else if (input instanceof LoggedNetworkString s) {
            s.setDefault(def);
        }
    }

    private static Object read(Object input, PortInfo port) {
        if (input instanceof LoggedNetworkNumber n) {
            double v = n.get();
            return switch (port.type()) {
                case INT -> (int) Math.round(v);
                default -> v;
            };
        }
        if (input instanceof LoggedNetworkBoolean b) {
            return b.get();
        }
        return ((LoggedNetworkString) input).get();
    }

    private static double number(PortInfo port, String def) {
        Object v = ParameterStore.typed(port, def.isEmpty() ? "0" : def);
        return ((Number) v).doubleValue();
    }
}
