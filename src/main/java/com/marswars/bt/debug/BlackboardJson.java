package com.marswars.bt.debug;

import com.google.gson.JsonElement;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;
import com.marswars.bt.core.Blackboard;
import java.util.Map;

/**
 * Converts blackboard entries to the btlive {@code {key: {type, value}}} shape. Numbers, booleans
 * and strings become JSON values, enums their name; anything else reports its type with a {@code
 * null} value (like BT.CPP entries without a JsonExporter converter). Types use BT.CPP spellings
 * where one exists ({@code double}, {@code int}, {@code bool}, {@code std::string}).
 */
final class BlackboardJson {
    private BlackboardJson() {}

    static JsonObject entries(Blackboard blackboard) {
        JsonObject entries = new JsonObject();
        for (Map.Entry<String, Object> e : blackboard.localEntries().entrySet()) {
            JsonObject entry = new JsonObject();
            entry.addProperty("type", typeName(e.getValue()));
            entry.add("value", value(e.getValue()));
            entries.add(e.getKey(), entry);
        }
        return entries;
    }

    static String typeName(Object v) {
        if (v == null) {
            return "";
        }
        if (v instanceof String) {
            return "std::string";
        }
        if (v instanceof Double || v instanceof Float) {
            return "double";
        }
        if (v instanceof Integer || v instanceof Short || v instanceof Byte) {
            return "int";
        }
        if (v instanceof Long) {
            return "int64_t";
        }
        if (v instanceof Boolean) {
            return "bool";
        }
        if (v instanceof Enum<?> en) {
            return en.getDeclaringClass().getSimpleName();
        }
        Class<?> c = v.getClass();
        return c.isAnonymousClass() || c.isSynthetic() || c.getSimpleName().isEmpty()
                ? c.getName()
                : c.getSimpleName();
    }

    static JsonElement value(Object v) {
        if (v instanceof Number n) {
            return new JsonPrimitive(n);
        }
        if (v instanceof Boolean b) {
            return new JsonPrimitive(b);
        }
        if (v instanceof String s) {
            return new JsonPrimitive(s);
        }
        if (v instanceof Enum<?> en) {
            return new JsonPrimitive(en.name());
        }
        return JsonNull.INSTANCE;
    }
}
