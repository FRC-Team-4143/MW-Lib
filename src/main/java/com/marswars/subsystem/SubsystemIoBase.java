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
     * Called by {@link SubsystemManager} when this IO must start or stop holding its outputs neutral
     * (its subsystem or the IO itself was switched off). Called on the change only, not every loop.
     * While neutral, {@code writeOutputs} must put every actuator in its configured brake or coast
     * instead of its last target. Every {@code *Mech} does this; a custom IO with actuators should
     * too, or it keeps applying its last command while its subsystem is disabled.
     *
     * @param neutral true to hold outputs neutral
     */
    public default void setNeutralOutputs(boolean neutral) {}
}
