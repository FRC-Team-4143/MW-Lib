package com.marswars.bt.core;

/**
 * Value type of a port. Drives literal parsing, the {@code type} attribute written to {@code
 * <TreeNodesModel>}, and robot-layer conventions (a TRAJECTORY port names a Choreo trajectory that
 * the auto pre-loads).
 */
public enum PortType {
    DOUBLE("double", Double.class),
    INT("int", Integer.class),
    BOOLEAN("bool", Boolean.class),
    STRING("std::string", String.class),
    ENUM("enum", String.class),
    TRAJECTORY("trajectory", String.class),
    ANY("", Object.class);

    private final String xml_type_;
    private final Class<?> java_type_;

    PortType(String xmlType, Class<?> javaType) {
        xml_type_ = xmlType;
        java_type_ = javaType;
    }

    /** Type string written to the XML model (BT.CPP spelling where one exists). */
    public String xmlType() {
        return xml_type_;
    }

    /** Java wrapper type used to validate literals of this port type. */
    public Class<?> javaType() {
        return java_type_;
    }
}
