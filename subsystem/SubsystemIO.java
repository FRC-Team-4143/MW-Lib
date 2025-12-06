package frc.mw_lib.subsystem;

public abstract class SubsystemIO<ConstantsType extends MwConstants> implements SubsystemIoBase {
  protected final ConstantsType CONSTANTS;
  private final String subsystem_name_;

  public SubsystemIO(ConstantsType constants) {
    CONSTANTS = constants;

    // Use some Java magic to pull the class name
    String name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);
    if (name.contains("IO")) {
      name = name.substring(0, name.indexOf("IO", 0));
    }

    subsystem_name_ = name;
  }

  /**
   * a common logging key to use for the subsystem
   *
   * @return the logging key to use as a base. eg getSubsystemKey() + "Some Special Value"
   */
  public String getSubsystemKey() {
    return "Subsystem/" + subsystem_name_ + "/";
  }
}
