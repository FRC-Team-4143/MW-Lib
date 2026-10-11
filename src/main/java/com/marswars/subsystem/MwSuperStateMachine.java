package com.marswars.subsystem;

import com.marswars.logging.MwLog;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;

/**
 * The robot's super state machine: one robot-level state that coordinates every subsystem state
 * machine. A robot's {@code RobotContainer} extends this class, registers its subsystems, and
 * requests subsystem states from {@link #updateLogic(double)}.
 *
 * <p>Each loop {@link #doControlLoop()} first runs this state machine (transition, then logic) and
 * then updates every registered subsystem, so the subsystem states requested here take effect in
 * the same loop.
 *
 * <p>Buttons and autos never talk to subsystems directly; they call {@link #setWantedState} on
 * the robot, the same way the robot calls it on subsystems. That replaces per-subsystem command
 * files: anything that involves more than one subsystem lives in one place, here, and teleop and
 * autos share it. Autos wait on {@link #getStateResult()} to know when a state finished or failed.
 *
 * <pre>{@code
 * public class RobotContainer extends MwSuperStateMachine<RobotStates> {
 *     public RobotContainer() {
 *         super(RobotStates.IDLE, BuildConstants.class);
 *         registerSubsystem(DrivetrainSubsystem.getInstance());
 *         registerSubsystem(ShooterSubsystem.getInstance());
 *         reset();
 *     }
 *
 *     @Override
 *     protected void updateLogic(double timestamp) {
 *         switch (getSystemState()) {
 *             case IDLE -> { drive.setWantedState(DriveStates.IDLE); shooter.setWantedState(ShooterStates.IDLE); }
 *             case DRIVE -> { drive.setWantedState(DriveStates.ARCADE); shooter.setWantedState(ShooterStates.IDLE); }
 *             case SHOOT -> { drive.setWantedState(DriveStates.ARCADE); shooter.setWantedState(ShooterStates.SHOOT); }
 *         }
 *     }
 * }
 *
 * // OI
 * driver.rightBumper().whileTrue(Commands.startEnd(
 *         () -> robot.setWantedState(RobotStates.SHOOT),
 *         () -> robot.setWantedState(RobotStates.DRIVE)));
 * }</pre>
 *
 * @param <S> the robot state enum
 */
public abstract class MwSuperStateMachine<S extends Enum<S>> extends SubsystemManager {
    /** Logging key prefix for the robot state, e.g. {@code Robot/State}. */
    public static final String LOG_KEY = "Robot/";

    // state info
    protected S system_state_;
    private S wanted_state_;
    private final S default_state_;
    private boolean wanted_failed_ = false;

    /**
     * @param default_state the state at startup and after {@link #reset()}
     * @param build_constants the generated BuildConstants class, for log metadata
     */
    public MwSuperStateMachine(S default_state, Object build_constants) {
        this(default_state, build_constants, List.of());
    }

    /**
     * @param default_state the state at startup and after {@link #reset()}
     * @param build_constants the generated BuildConstants class, for log metadata
     * @param disabled_subsystems names of subsystems that should not be registered
     */
    public MwSuperStateMachine(
            S default_state, Object build_constants, List<String> disabled_subsystems) {
        this(default_state, build_constants, disabled_subsystems, true);
    }

    /** Unit tests only: {@code start_logging = false} skips starting AdvantageKit. */
    MwSuperStateMachine(
            S default_state,
            Object build_constants,
            List<String> disabled_subsystems,
            boolean start_logging) {
        super(build_constants, disabled_subsystems, start_logging);
        default_state_ = default_state;
        system_state_ = default_state;
        wanted_state_ = default_state;
    }

    /**
     * Moves from the current state toward the wanted state. Called once per loop before {@link
     * #updateLogic(double)}. The default goes straight to the wanted state. Override it as the
     * transition table: switch on the current state, then on {@code wanted}; every condition lives
     * here (guards, step states, values decided from sensors into fields that {@link #updateLogic}
     * reads), and nothing else does. There are no enter/exit actions: a state asks for what it
     * needs every loop.
     *
     * @param wanted the state that was last requested
     */
    protected void handleStateTransition(S wanted) {
        system_state_ = wanted;
    }

    /**
     * Requests subsystem states for the current robot state. Called once per loop, after the
     * transition and before every subsystem update.
     *
     * @param timestamp the loop timestamp in seconds
     */
    protected abstract void updateLogic(double timestamp);

    /** DO NOT OVERRIDE. Runs the robot state machine before the subsystems update. */
    @Override
    protected final void update(double timestamp) {
        MwLog.log(LOG_KEY + "WantedState", wanted_state_);
        handleStateTransition(wanted_state_);

        MwLog.log(LOG_KEY + "State", system_state_);
        updateLogic(timestamp);
    }

    /** Resets the robot state to the default state, then resets every subsystem. */
    @Override
    public void reset() {
        system_state_ = default_state_;
        wanted_state_ = default_state_;
        wanted_failed_ = false;
        super.reset();
    }

    /**
     * Sets the wanted robot state. It stays wanted until something else asks for another state.
     *
     * @param wanted the state to move to
     */
    public void setWantedState(S wanted) {
        wanted_state_ = wanted;
        wanted_failed_ = false;
    }

    /**
     * Gets the most recently requested robot state
     *
     * @return the wanted state
     */
    public S getWantedState() {
        return wanted_state_;
    }

    /**
     * Gets the current robot state
     *
     * @return the current state
     */
    public S getSystemState() {
        return system_state_;
    }

    /**
     * How the wanted state is going, for autos that wait on it (the {@code SetWantedState}
     * behavior-tree node): {@link StateResult#FAILURE} after {@link #failWantedState()}, {@link
     * StateResult#SUCCESS} once the robot is in the wanted state, otherwise {@link
     * StateResult#RUNNING}.
     *
     * <p>A multi-step sequence (aim, spin up, feed) is a wanted state whose transition walks the
     * robot through step states first; it reports SUCCESS when it reaches the wanted state itself.
     *
     * @return the result of the most recent request
     */
    public StateResult getStateResult() {
        if (wanted_failed_) {
            return StateResult.FAILURE;
        }
        return system_state_ == wanted_state_ ? StateResult.SUCCESS : StateResult.RUNNING;
    }

    /**
     * Call from {@link #handleStateTransition} or {@link #updateLogic} when the wanted state cannot
     * be reached (a timeout, a missing game piece). {@link #getStateResult()} reports FAILURE until
     * the next {@link #setWantedState}. What the robot does next is up to the caller.
     */
    protected void failWantedState() {
        wanted_failed_ = true;
    }

    /**
     * @param state the state to check
     * @return true if the robot is currently in {@code state}
     */
    public boolean isIn(S state) {
        return system_state_ == state;
    }

    /**
     * @param state the state to watch
     * @return a trigger that is true while the robot is in {@code state}
     */
    public Trigger inState(S state) {
        return new Trigger(() -> isIn(state));
    }

    /** Result of a wanted state, see {@link #getStateResult()}. */
    public enum StateResult {
        RUNNING,
        SUCCESS,
        FAILURE
    }
}
