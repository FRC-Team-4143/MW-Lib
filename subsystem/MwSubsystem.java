package frc.mw_lib.subsystem;

import dev.doglog.DogLog;

public abstract class MwSubsystem<
        StateType extends Enum<StateType>,
        ConstantsType extends MwConstants>
    implements MwSubsystemBase {

  // state info
  protected StateType system_state_;
  protected final ConstantsType CONSTANTS;
  private StateType wanted_state_;

  // internal info
  protected String subsystem_name_;

  public MwSubsystem(StateType default_state, ConstantsType constants) {
    // Use some Java magic to pull the class name
    String name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);
    if (name.endsWith("Subsystem")) {
      name = name.substring(0, name.length() - "Subsystem".length());
    }

    CONSTANTS = constants;

    subsystem_name_ = name;

    system_state_ = default_state;
    wanted_state_ = default_state;
  }

  protected void handleStateTransition(StateType wanted) {
    system_state_ = wanted_state_;
  }

  /** Computes updated outputs for the actuators */
  public abstract void updateLogic(double timestamp);

  /** Called to reset and configure the subsystem */
  public abstract void reset();

  /** Functions below this line are common. they can be used on all subsystems */

  /**
   * Sets the desired state for the state machine
   *
   * @param wanted the desired state machine state to transition to
   */
  public void setWantedState(StateType wanted) {
    wanted_state_ = wanted;
    DogLog.log(getSubsystemKey() + "WantedState", wanted_state_);
  }

  /**
   * DO NOT OVERRIDE THIS METHOD IN THE BASE CLASS IF YOU DO, BAD THINGS WILL HAPPEN! YOUR SUBSYSTEM
   * WILL NOT TICK CORRECTLY
   */
  @Override
  public void update(double timestamp) {
    handleStateTransition(wanted_state_);

    updateLogic(timestamp);
  }

  /**
   * a common logging key to use for the subsystem
   *
   * @return the logging key to use as a base. eg getSubsystemKey() + "Some Special Value"
   */
  public String getSubsystemKey() {
    return "Subsystem/" + subsystem_name_ + "/";
  }

  /**
   * a common NT key to use for the subsystem
   *
   * @return the NT key to use as a base. eg getSubsystemKey() + "Some Special Value"
   */
  public String getNtKey() {
    return "Robot/Subsystem/" + subsystem_name_ + "/";
  }

  /**
   * Gets the name of the subsystem used to enable and disable. Also used for logging
   *
   * @return the string name of the subsystem
   */
  public String getName() {
    return subsystem_name_;
  }
}
