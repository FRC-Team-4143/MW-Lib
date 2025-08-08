package frc.mw_lib.subsystem;

import monologue.Logged;

public abstract class MWSubsystem implements Logged {

  /** Reads all sensors and stores periodic data */
  public abstract void readPeriodicInputs(double timestamp);

  /** Computes updated outputs for the actuators */
  public abstract void updateLogic(double timestamp);

  /** Writes the periodic outputs to actuators (motors and etc...) */
  public abstract void writePeriodicOutputs(double timestamp);

  /** Outputs all logging information to the SmartDashboard */
  public abstract void outputTelemetry(double timestamp);

  /** Called to reset and configure the subsystem */
  public abstract void reset();
}
