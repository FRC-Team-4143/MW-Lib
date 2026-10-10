package com.marswars.bt.core;

import java.util.HashMap;
import java.util.Map;

/**
 * Source of tree parameter values that live outside any single tree run, such as values tuned
 * from a dashboard. A tree parameter is a port declared for a {@code <BehaviorTree>} in {@code
 * <TreeNodesModel>} (a {@code <SubTree ID="...">} model), e.g.
 *
 * <pre>{@code
 * <SubTree ID="CitrusSynergy">
 *   <input_port name="middle_wait_msec" type="int" default="3000">Wait for partners</input_port>
 * </SubTree>
 * }</pre>
 *
 * <p>Nodes read it like any blackboard entry: {@code <Delay delay_msec="{middle_wait_msec}">}.
 */
@FunctionalInterface
public interface ParameterStore {
    /**
     * Current value of {@code key}, typed by {@code port.type()} (Double, Integer, Boolean or
     * String). Creates the parameter with the port's default on first use.
     */
    Object value(String key, PortInfo port);

    /** Typed value of a port literal: Double, Integer, Boolean, or the string itself. */
    static Object typed(PortInfo port, String literal) {
        if (literal == null) {
            return null;
        }
        String context = "parameter '" + port.name() + "'";
        return switch (port.type()) {
            case DOUBLE -> PortValues.parseLiteral(literal, Double.class, context);
            case INT -> PortValues.parseLiteral(literal, Integer.class, context);
            case BOOLEAN -> PortValues.parseLiteral(literal, Boolean.class, context);
            default -> literal;
        };
    }

    /** Parameters held in memory (tests, tools); change them with {@link #set}. */
    final class InMemory implements ParameterStore {
        private final Map<String, Object> values_ = new HashMap<>();

        @Override
        public synchronized Object value(String key, PortInfo port) {
            return values_.computeIfAbsent(key, k -> typed(port, port.defaultValue()));
        }

        public synchronized void set(String key, Object value) {
            values_.put(key, value);
        }

        public synchronized Map<String, Object> snapshot() {
            return Map.copyOf(values_);
        }
    }
}
