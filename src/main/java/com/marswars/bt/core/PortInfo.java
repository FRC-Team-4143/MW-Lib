package com.marswars.bt.core;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Declaration of one port of a node type: its name, direction, value type, optional default,
 * description, and (for enum-like ports) the allowed choices. Ports are set in XML as attributes.
 *
 * @param name attribute name
 * @param direction input/output/inout
 * @param type value type
 * @param defaultValue literal default, or {@code null} when the port is required
 * @param description human-readable description (goes into {@code <TreeNodesModel>})
 * @param choices allowed values for ENUM ports (empty otherwise)
 * @param enumClass Java enum backing an ENUM port, or {@code null}
 */
public record PortInfo(
        String name,
        PortDirection direction,
        PortType type,
        String defaultValue,
        String description,
        List<String> choices,
        Class<?> enumClass) {

    public PortInfo {
        Objects.requireNonNull(name, "port name");
        Objects.requireNonNull(direction, "port direction");
        Objects.requireNonNull(type, "port type");
        description = description == null ? "" : description;
        choices = choices == null ? List.of() : List.copyOf(choices);
    }

    public static PortInfo input(String name, PortType type, String description) {
        return new PortInfo(name, PortDirection.INPUT, type, null, description, List.of(), null);
    }

    public static PortInfo input(String name, PortType type, String defaultValue, String description) {
        return new PortInfo(
                name, PortDirection.INPUT, type, defaultValue, description, List.of(), null);
    }

    public static PortInfo output(String name, PortType type, String description) {
        return new PortInfo(name, PortDirection.OUTPUT, type, null, description, List.of(), null);
    }

    public static PortInfo inout(String name, PortType type, String description) {
        return new PortInfo(name, PortDirection.INOUT, type, null, description, List.of(), null);
    }

    /** Input port whose literal must be one of the enum's constant names. */
    public static <E extends Enum<E>> PortInfo enumInput(
            String name, Class<E> enumClass, E defaultValue, String description) {
        List<String> names = new ArrayList<>();
        for (E e : enumClass.getEnumConstants()) {
            names.add(e.name());
        }
        return new PortInfo(
                name,
                PortDirection.INPUT,
                PortType.ENUM,
                defaultValue == null ? null : defaultValue.name(),
                description,
                names,
                enumClass);
    }

    /** Input port restricted to an explicit list of names (no Java enum behind it). */
    public static PortInfo choiceInput(
            String name, List<String> choices, String defaultValue, String description) {
        return new PortInfo(
                name, PortDirection.INPUT, PortType.ENUM, defaultValue, description, choices, null);
    }

    /** Input port naming a Choreo trajectory; the auto pre-loads every literal value it finds. */
    public static PortInfo trajectory(String name, String description) {
        return new PortInfo(
                name, PortDirection.INPUT, PortType.TRAJECTORY, null, description, List.of(), null);
    }

    /** A port that must be present in the XML: not an output, and without a default. */
    public boolean required() {
        return direction != PortDirection.OUTPUT && defaultValue == null;
    }

    /** Type string for {@code <TreeNodesModel>}: enum ports use the enum's simple name. */
    public String xmlType() {
        if (type == PortType.ENUM && enumClass != null) {
            return enumClass.getSimpleName();
        }
        return type.xmlType();
    }
}
