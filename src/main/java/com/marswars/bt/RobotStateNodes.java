package com.marswars.bt;

import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.NodeKind;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.StatefulActionNode;
import com.marswars.subsystem.MwSuperStateMachine;
import com.marswars.subsystem.MwSuperStateMachine.StateResult;
import java.util.List;
import java.util.function.Supplier;

/**
 * Nodes for a robot's super state machine ({@link MwSuperStateMachine}, usually the {@code
 * RobotContainer}), so autos use the same robot states (and scoring sequences) as teleop:
 *
 * <ul>
 *   <li>{@code SetWantedState}: calls {@code setWantedState(state)} on the robot, then stays
 *       RUNNING until {@link MwSuperStateMachine#getStateResult()} is SUCCESS (the robot is in the
 *       state) or FAILURE (the state machine gave up, or something else asked for another state).
 *       Wrap it in {@code Timeout}, {@code Fallback} or {@code Retry} for auto-only fail handling.
 *   <li>condition {@code IsSystemState}: the robot is in a state
 * </ul>
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
            Supplier<? extends MwSuperStateMachine<S>> robot,
            Class<S> state_class) {
        factory.registerNodeType(
                "SetWantedState",
                NodeKind.ACTION,
                "Request a robot state and wait until the robot is in it (SUCCESS) or the state"
                        + " fails (FAILURE)",
                List.of(PortInfo.enumInput("state", state_class, null, "Robot state to request")),
                (name, cfg) ->
                        new StatefulActionNode(name, cfg) {
                            private S state_;

                            @Override
                            protected NodeStatus onStart() {
                                state_ = getEnum("state", state_class);
                                robot.get().setWantedState(state_);
                                return onRunning();
                            }

                            @Override
                            protected NodeStatus onRunning() {
                                MwSuperStateMachine<S> sm = robot.get();
                                if (sm.getWantedState() != state_) {
                                    return NodeStatus.FAILURE; // someone asked for another state
                                }
                                StateResult result = sm.getStateResult();
                                return switch (result) {
                                    case SUCCESS -> NodeStatus.SUCCESS;
                                    case FAILURE -> NodeStatus.FAILURE;
                                    case RUNNING -> NodeStatus.RUNNING;
                                };
                            }

                            @Override
                            protected void onHalted() {}
                        });
        factory.registerSimpleCondition(
                "IsSystemState",
                "The robot is in the given state",
                List.of(PortInfo.enumInput("state", state_class, null, "Robot state to check")),
                n -> robot.get().isIn(n.getEnum("state", state_class)));
    }
}
