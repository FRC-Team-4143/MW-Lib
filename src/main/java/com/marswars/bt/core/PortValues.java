package com.marswars.bt.core;

import java.util.Arrays;
import java.util.Locale;

/** Helpers for port strings: blackboard pointer syntax ({@code {key}}) and literal parsing. */
public final class PortValues {
    private PortValues() {}

    /** True when {@code raw} is a blackboard pointer such as {@code {target}} or {@code {@root}}. */
    public static boolean isPointer(String raw) {
        return raw != null && raw.length() >= 3 && raw.startsWith("{") && raw.endsWith("}");
    }

    /** The key inside a pointer: {@code {target}} gives {@code target}. */
    public static String pointerKey(String raw) {
        return raw.substring(1, raw.length() - 1);
    }

    /** Like {@link #pointerKey(String)}, resolving the {@code {=}} shorthand to {@code portName}. */
    public static String pointerKey(String raw, String portName) {
        String key = pointerKey(raw);
        return "=".equals(key) ? portName : key;
    }

    /** Converts a blackboard value to the requested wrapper/enum type, parsing strings. */
    @SuppressWarnings("unchecked")
    public static <T> T convert(Object value, Class<T> type, String context) {
        if (value == null) {
            return null;
        }
        if (type == Object.class || type.isInstance(value)) {
            return (T) value;
        }
        if (value instanceof String s) {
            return parseLiteral(s, type, context);
        }
        if (value instanceof Number n) {
            if (type == Double.class) {
                return (T) Double.valueOf(n.doubleValue());
            }
            if (type == Integer.class) {
                return (T) Integer.valueOf(n.intValue());
            }
            if (type == Long.class) {
                return (T) Long.valueOf(n.longValue());
            }
        }
        if (type == String.class) {
            return (T) value.toString();
        }
        throw new BtException(
                "Cannot convert "
                        + context
                        + " of type "
                        + value.getClass().getSimpleName()
                        + " to "
                        + type.getSimpleName());
    }

    /** Parses an XML attribute literal into the requested wrapper/enum type. */
    @SuppressWarnings({"unchecked", "rawtypes"})
    public static <T> T parseLiteral(String raw, Class<T> type, String context) {
        try {
            if (type == String.class || type == Object.class) {
                return (T) raw;
            }
            String trimmed = raw.trim();
            if (type == Double.class) {
                return (T) Double.valueOf(trimmed);
            }
            if (type == Integer.class) {
                return (T) Integer.valueOf(trimmed);
            }
            if (type == Long.class) {
                return (T) Long.valueOf(trimmed);
            }
            if (type == Boolean.class) {
                return (T) parseBoolean(trimmed, context);
            }
            if (type.isEnum()) {
                try {
                    return (T) Enum.valueOf((Class) type, trimmed);
                } catch (IllegalArgumentException e) {
                    throw new BtException(
                            "Invalid "
                                    + context
                                    + " value '"
                                    + raw
                                    + "'; expected one of "
                                    + Arrays.toString(type.getEnumConstants()));
                }
            }
        } catch (NumberFormatException e) {
            throw new BtException(
                    "Cannot parse " + context + " value '" + raw + "' as " + type.getSimpleName(),
                    e);
        }
        throw new BtException("Unsupported port type " + type.getName() + " for " + context);
    }

    private static Boolean parseBoolean(String raw, String context) {
        switch (raw.toLowerCase(Locale.ROOT)) {
            case "true", "1", "yes":
                return Boolean.TRUE;
            case "false", "0", "no":
                return Boolean.FALSE;
            default:
                throw new BtException(
                        "Cannot parse " + context + " value '" + raw + "' as a boolean");
        }
    }
}
