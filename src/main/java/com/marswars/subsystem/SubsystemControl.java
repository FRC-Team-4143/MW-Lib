package com.marswars.subsystem;

/**
 * Who drives a subsystem. Every registered subsystem has one, set live from a dashboard at {@code
 * /MwLib/Subsystems/<Name>/Control} or from code with {@link SubsystemManager#requestControl}.
 * Every boot starts at {@link #ROBOT}.
 */
public enum SubsystemControl {
    /**
     * The default. The subsystem follows {@code setWantedState} requests from the super state
     * machine, autos and buttons.
     */
    ROBOT,

    /**
     * The subsystem is held in the state named at {@code /MwLib/Subsystems/<Name>/PinnedState}. Its
     * own transition and logic run as usual; every {@code setWantedState} request is dropped and
     * logged. Ignored while the FMS is attached.
     */
    PINNED,

    /**
     * Tuning. The subsystem's logic does not run and every {@code setWantedState} request is
     * dropped. Each mechanism follows its manual output at {@code
     * /MwLib/Subsystems/<Name>/IO/<Mech>/Manual/*}, and its mechanisms' tunables (gains) accept
     * changes. On entry, position mechanisms hold where they are and velocity mechanisms start at
     * zero. Ignored while the FMS is attached.
     */
    MANUAL,

    /**
     * The subsystem's logic does not run and every IO puts its outputs in neutral (each motor's
     * configured brake or coast). Inputs are still read and logged. Wanted-state requests are
     * dropped and logged.
     */
    OFF
}
