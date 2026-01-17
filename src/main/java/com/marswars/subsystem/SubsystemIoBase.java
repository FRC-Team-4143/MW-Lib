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
}
