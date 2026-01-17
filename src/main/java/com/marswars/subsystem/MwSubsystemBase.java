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
}
