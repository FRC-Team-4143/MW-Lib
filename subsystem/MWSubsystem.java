package frc.mw_lib.subsystem;

import dev.doglog.DogLog;

public abstract class MWSubsystem<IoType extends SubsystemIO, StateType extends Enum<StateType>>
    implements MWSubsystemBase {

  // state info
  protected IoType io;
  protected StateType system_state_;
  protected StateType wanted_state_;

  // internal info
  protected String subsystem_name_;

  public MWSubsystem() {
    // Use some Java magic to pull the class name
    String name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);
    if (name.endsWith("Subsystem")) {
      name = name.substring(0, name.length() - "Subsystem".length());
    }

    subsystem_name_ = name;
  }

  public void handleStateTransition(StateType wanted) {}

  /** Computes updated outputs for the actuators */
  public abstract void updateLogic(double timestamp);

  /** Called to reset and configure the subsystem */
  public abstract void reset();

  /** Functions below this line are common. they can be used on all subsystems */

  /** Gets the name of the subsystem used to enable and disable. Also used for logging */
  public String getName() {
    return subsystem_name_;
  }

  /**
   * Gets the I/O container for the manager to exercise
   *
   * @return the generic I/O container to be called
   */
  public IoType getIo() {
    return io;
  }

  /**
   * Sets the desired state for the state machine
   *
   * @param wanted the desired state machine state to transition to
   */
  public void setWantedState(StateType wanted) {
    wanted_state_ = wanted;
    DogLog.log(getSubsystemKey() + "Wanted State", wanted_state_);

    if (system_state_ != wanted) {
      handleStateTransition(wanted);
    }
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
