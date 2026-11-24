package com.marswars.subsystem;

/**
 * Output modes a mechanism can be driven in while its subsystem is {@link SubsystemControl#MANUAL}.
 * Each mechanism lists the ones it supports in {@link ControllableIo#getManualModes()}; the setpoint
 * is in the units of the matching {@code setTarget*} method.
 */
public enum ManualMode {
    /** Motors in their configured brake or coast. */
    NEUTRAL,
    POSITION,
    MOTION_PROFILE_POSITION,
    VELOCITY,
    MOTION_PROFILE_VELOCITY,
    /** Clamped to {@link SubsystemManager#setManualDutyCycleLimit(double)}. */
    DUTY_CYCLE,
    CURRENT
}
