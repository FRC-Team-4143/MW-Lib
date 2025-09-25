package frc.mw_lib.subsystem;

public abstract class MWSubsystem<IoType extends SubsystemIO> {

  protected IoType io;

  public IoType getIo() {
    return io;
  }

  /** Computes updated outputs for the actuators */
  public abstract void updateLogic(double timestamp);

  /** Called to reset and configure the subsystem */
  public abstract void reset();
}
