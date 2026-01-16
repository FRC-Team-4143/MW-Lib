package com.marswars.subsystem;

public interface SubsystemIoBase {

  public abstract void readInputs(double timestamp);

  public abstract void writeOutputs(double timestamp);

  public abstract void logData();

}
