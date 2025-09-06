package frc.mw_lib.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class MWSubsystem extends SubsystemBase {

  protected SubsystemIO io;

  /** Computes updated outputs for the actuators */
  public abstract void updateLogic(double timestamp);

  /** Called to reset and configure the subsystem */
  public abstract void reset();
}
