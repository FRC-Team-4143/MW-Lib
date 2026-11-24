package com.marswars.subsystem;

import java.util.List;

public interface MwSubsystemBase {
    public List<SubsystemIoBase> getIos();

    /** Returns the name of the subsystem */
    public String getName();

    /** Returns the unique key for this subsystem for logging purposes */
    public String getSubsystemKey();

    /** Computes updated outputs for the actuators */
    public void update(double timestamp);

    /** Called to reset and configure the subsystem */
    public void reset();

    /**
     * Called by {@link SubsystemManager} at the top of every loop with the control it decided on.
     * Subsystems that are not an {@link MwSubsystem} can ignore it: the manager still skips {@link
     * #update(double)} and neutralizes outputs while {@link SubsystemControl#OFF}.
     *
     * @param control the control in effect this loop
     * @param pinned_state the name of the pinned state when {@code control} is {@link
     *     SubsystemControl#PINNED}, otherwise null
     */
    public default void applyControl(SubsystemControl control, String pinned_state) {}

    /**
     * The names of the states this subsystem can be pinned to. Empty means it can't be pinned.
     *
     * @return the state names
     */
    public default List<String> getStateNames() {
        return List.of();
    }
}
