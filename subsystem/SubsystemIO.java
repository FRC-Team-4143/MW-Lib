package frc.mw_lib.subsystem;

public interface SubsystemIO {

  public default void readInputs(double timestamp) {}

  public default void writeOutputs(double timestamp) {}
}
