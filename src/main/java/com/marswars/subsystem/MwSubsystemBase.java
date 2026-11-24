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
     * Called once when the subsystem goes back to {@link SubsystemMode#ENABLED} after being disabled
     * or in tuning, before its next update. Defaults to {@link #reset()}.
     */
    public default void onEnable() {
        reset();
    }
}
