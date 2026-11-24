package com.marswars.subsystem;

import com.marswars.logging.MwLog;

import java.util.Arrays;
import java.util.List;

public abstract class MwSubsystem<
                StateType extends Enum<StateType>, ConstantsType extends MwConstants>
        implements MwSubsystemBase {

    // state info
    protected StateType system_state_;
    protected final ConstantsType CONSTANTS;
    private StateType wanted_state_;
    private final StateType default_state_;
    private final Class<StateType> state_class_;

    // who drives this subsystem; applied by SubsystemManager at the top of each loop
    private SubsystemControl control_ = SubsystemControl.ROBOT;
    private StateType pinned_state_ = null;

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

        // set initial states
        system_state_ = default_state;
        wanted_state_ = default_state;
        default_state_ = default_state;
        state_class_ = default_state.getDeclaringClass();
    }

    protected void handleStateTransition(StateType wanted) {
        system_state_ = wanted;
    }

    /** Computes updated outputs for the actuators */
    public abstract void updateLogic(double timestamp);

    /** Called to reset and configure the subsystem */
    public abstract void reset();

    /** Functions below this line are common. they can be used on all subsystems */

    /**
     * Sets the desired state for the state machine. Dropped (and logged as {@code
     * DroppedWantedState}) unless the subsystem's control is {@link SubsystemControl#ROBOT}.
     *
     * @param wanted the desired state machine state to transition to
     */
    public void setWantedState(StateType wanted) {
        if (control_ != SubsystemControl.ROBOT) {
            MwLog.log(getSubsystemKey() + "DroppedWantedState", wanted);
            return;
        }
        wanted_state_ = wanted;
    }

    /**
     * Gets the most recently accepted wanted state (the pinned state while pinned)
     *
     * @return the wanted state
     */
    public StateType getWantedState() {
        return wanted_state_;
    }

    /**
     * Gets who currently drives this subsystem
     *
     * @return the control in effect this loop
     */
    public SubsystemControl getControl() {
        return control_;
    }

    /**
     * @return true if the subsystem follows {@link #setWantedState} requests, i.e. its control is
     *     {@link SubsystemControl#ROBOT}
     */
    public boolean isAvailable() {
        return control_ == SubsystemControl.ROBOT;
    }

    /**
     * Applied by {@link SubsystemManager} at the top of each loop. Leaving {@link
     * SubsystemControl#OFF} or {@link SubsystemControl#MANUAL} for a control that runs the logic,
     * or going back to {@link SubsystemControl#ROBOT}, calls {@link #reset()} and returns the
     * wanted state to the default so stale state is never resumed.
     */
    @Override
    public final void applyControl(SubsystemControl control, String pinned_state) {
        SubsystemControl previous = control_;
        control_ = control;
        pinned_state_ =
                control == SubsystemControl.PINNED ? Enum.valueOf(state_class_, pinned_state) : null;

        boolean resume =
                control != SubsystemControl.OFF
                        && control != SubsystemControl.MANUAL
                        && previous != control
                        && (previous == SubsystemControl.OFF
                                || previous == SubsystemControl.MANUAL
                                || control == SubsystemControl.ROBOT);
        if (resume) {
            reset();
            wanted_state_ = default_state_;
        }
    }

    /** {@inheritDoc} */
    @Override
    public List<String> getStateNames() {
        return Arrays.stream(state_class_.getEnumConstants()).map(Enum::name).toList();
    }

    /**
     * Gets the current state of the state machine
     *
     * @return the current state of the state machine
     */
    public StateType getSystemState() {
        return system_state_;
    }

    /**
     * DO NOT OVERRIDE THIS METHOD IN THE BASE CLASS IF YOU DO, BAD THINGS WILL HAPPEN! YOUR
     * SUBSYSTEM WILL NOT TICK CORRECTLY
     */
    @Override
    public void update(double timestamp) {
        if (control_ == SubsystemControl.PINNED) {
            wanted_state_ = pinned_state_;
        }

        // log the wanted state before we handle the transition
        MwLog.log(getSubsystemKey() + "WantedState", wanted_state_);
        handleStateTransition(wanted_state_);

        // log the current state before we run the logic
        MwLog.log(getSubsystemKey() + "State", system_state_);
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
