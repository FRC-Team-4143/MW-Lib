package com.marswars.bt;

import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.PortInfo;
import com.marswars.subsystem.MwSuperStateMachine;
import java.util.List;
import java.util.function.Supplier;

/**
 * Nodes for a robot's super state machine ({@link MwSuperStateMachine}, usually the {@code
 * RobotContainer}), so autos use the same robot states (and scoring sequences) as teleop:
 *
 * <ul>
 *   <li>{@code SetWantedState}: calls {@code setWantedState(state)} on the robot and succeeds at
 *       once. A request is instant, like a button press; the state stays wanted until something
 *       asks for another one.
 *   <li>condition {@code IsSystemState}: the robot is in a state
 * </ul>
 *
 * To wait until the robot reaches a state, retry the condition (add a {@code Timeout} around it so
 * it can't wait forever):
 *
 * <pre>{@code
 * <SetWantedState state="SCORE"/>
 * <Timeout msec="3000">
 *   <RetryUntilSuccessful num_attempts="-1"><IsSystemState state="SCORE"/></RetryUntilSuccessful>
 * </Timeout>
 * }</pre>
 *
 * <pre>{@code
 * RobotStateNodes.register(factory, RobotContainer::getInstance, RobotStates.class);
 * }</pre>
 *
 * The state enum is the robot's own, so these nodes are listed with the robot's nodes in node-spec
 * files, not in MW-Lib's bundled spec. The supplier is only called while a tree runs.
 */
public final class RobotStateNodes {
    private RobotStateNodes() {}

    public static <S extends Enum<S>> void register(
            BehaviorTreeFactory factory,
            Supplier<? extends MwSuperStateMachine<S, ?>> robot,
            Class<S> state_class) {
        factory.registerSetState(
                "SetWantedState",
                "Request a robot state (instant: succeeds at once)",
                state_class,
                s -> robot.get().setWantedState(s));
        factory.registerSimpleCondition(
                "IsSystemState",
                "The robot is in the given state",
                List.of(PortInfo.enumInput("state", state_class, null, "Robot state to check")),
                n -> robot.get().isIn(n.getEnum("state", state_class)));
    }
}
