package com.marswars.subsystem;

import java.util.List;

public interface MwSubsystemBase {
  public List<SubsystemIoBase> getIos();

  public String getName();

  public String getSubsystemKey();

  /** Computes updated outputs for the actuators */
  public void update(double timestamp);

  /** Called to reset and configure the subsystem */
  public void reset();
}
