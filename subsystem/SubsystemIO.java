package frc.mw_lib.subsystem;

public abstract class SubsystemIO <ConstantsType extends MWConstants> {
  protected final ConstantsType CONSTANTS;

  public SubsystemIO(ConstantsType constants){
    CONSTANTS = constants;
  }

  public abstract void readInputs(double timestamp);

  public abstract void writeOutputs(double timestamp);
}
