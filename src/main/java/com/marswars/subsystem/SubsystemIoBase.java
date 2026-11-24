package com.marswars.subsystem;

public interface SubsystemIoBase {

    /**
     * Read inputs from hardware or simulation
     *
     * @param timestamp the current timestamp
     */
    public abstract void readInputs(double timestamp);

    /**
     * Write outputs to hardware or simulation
     *
     * @param timestamp the current timestamp
     */
    public abstract void writeOutputs(double timestamp);

    /** Log data to NetworkTables or other logging systems */
    public abstract void logData();

    /**
     * Set by {@link SubsystemManager} every loop before {@link #writeOutputs(double)}. While true,
     * {@code writeOutputs} must put every actuator in neutral (its configured brake or coast)
     * instead of applying its last target. Used when the subsystem is {@link SubsystemControl#OFF}.
     * IO with no actuators can ignore it.
     *
     * @param neutral true to hold outputs in neutral
     */
    public default void setNeutralOutputs(boolean neutral) {}
}
