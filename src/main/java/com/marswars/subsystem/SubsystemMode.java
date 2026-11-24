package com.marswars.subsystem;

/**
 * A subsystem's live mode, set from {@code /MwLib/Subsystems/<Name>/Mode} or {@link
 * SubsystemManager#requestMode}. Every boot starts {@link #ENABLED}.
 */
public enum SubsystemMode {
    /** Normal: the subsystem's logic runs every loop. */
    ENABLED,

    /**
     * The subsystem's logic does not run and its IO holds outputs neutral (each motor's configured
     * brake or coast). Inputs are still read and logged.
     */
    DISABLED,

    /**
     * The subsystem's logic does not run, so nothing but you moves it: its mechanisms hold where
     * they were, then follow their {@code /Tuning/.../Setpoint} entries. Refused while the FMS is
     * attached.
     */
    TUNING
}
