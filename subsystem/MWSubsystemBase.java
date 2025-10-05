package frc.mw_lib.subsystem;

public interface MWSubsystemBase {
  public SubsystemIO getIo();

  public String getName();

  public String getSubsystemKey();

  /** Computes updated outputs for the actuators */
  public void update(double timestamp);

  /** Called to reset and configure the subsystem */
  public void reset();
}
