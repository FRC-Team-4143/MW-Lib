package com.marswars.bt.swerve;

import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.NodeKind;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.PortType;
import com.marswars.bt.core.SimpleActionNode;
import com.marswars.bt.core.SimpleConditionNode;
import com.marswars.bt.core.StatefulActionNode;
import com.marswars.swerve_lib.MwSwerveSubsystem;
import com.marswars.swerve_lib.SwerveStates;
import java.util.List;
import java.util.function.Supplier;

/**
 * MW-Lib's shared swerve nodes for any robot built on {@link MwSwerveSubsystem}:
 *
 * <ul>
 *   <li>{@code FollowTrajectory}: drive a Choreo path (see {@link FollowTrajectoryNode})
 *   <li>{@code WaitForChoreoEvent}: RUNNING until the running path passes an event marker
 *   <li>{@code SetSwerveState}: request a swerve state
 *   <li>conditions {@code IsAtChoreoSetpoint}, {@code HasChoreoTimeElapsed}, {@code
 *       HasChoreoEventPassed}, {@code IsChassisStationary}
 * </ul>
 *
 * <pre>{@code
 * SwerveNodes.register(factory, SwerveSubsystem::getInstance);
 * }</pre>
 *
 * The supplier is only called while a tree runs, never while trees are built or validated.
 */
public final class SwerveNodes {
    private SwerveNodes() {}

    public static void register(
            BehaviorTreeFactory factory, Supplier<? extends MwSwerveSubsystem<?>> swerve) {
        factory.registerLibraryNode(
                FollowTrajectoryNode.ID,
                NodeKind.ACTION,
                "Follow a Choreo trajectory until its end setpoint is reached",
                FollowTrajectoryNode.PORTS,
                (name, cfg) -> new FollowTrajectoryNode(name, cfg, swerve));
        factory.registerLibraryNode(
                "WaitForChoreoEvent",
                NodeKind.ACTION,
                "Wait until the running trajectory passes a Choreo event marker (use next to a"
                        + " FollowTrajectory in a ParallelDeadline)",
                List.of(PortInfo.input("event", PortType.STRING, "Choreo event marker name")),
                (name, cfg) ->
                        new StatefulActionNode(name, cfg) {
                            @Override
                            protected NodeStatus onStart() {
                                return onRunning();
                            }

                            @Override
                            protected NodeStatus onRunning() {
                                return swerve.get().hasChoreoEventBeenPassed(getString("event"))
                                        ? NodeStatus.SUCCESS
                                        : NodeStatus.RUNNING;
                            }

                            @Override
                            protected void onHalted() {}
                        });
        factory.registerLibraryNode(
                "SetSwerveState",
                NodeKind.ACTION,
                "Request a swerve state",
                List.of(PortInfo.enumInput("state", SwerveStates.class, null, "State to request")),
                (name, cfg) ->
                        new SimpleActionNode(
                                name,
                                cfg,
                                n -> {
                                    swerve.get()
                                            .setWantedState(n.getEnum("state", SwerveStates.class));
                                    return NodeStatus.SUCCESS;
                                }));
        condition(
                factory,
                "IsAtChoreoSetpoint",
                "The swerve reached the end of its Choreo trajectory",
                List.of(),
                n -> swerve.get().isAtChoreoSetpoint());
        condition(
                factory,
                "HasChoreoTimeElapsed",
                "At least `seconds` of the current Choreo trajectory have run",
                List.of(PortInfo.input("seconds", PortType.DOUBLE, "Seconds on the path")),
                n -> swerve.get().hasChoreoTimeElapsed(n.getDouble("seconds")));
        condition(
                factory,
                "HasChoreoEventPassed",
                "The running trajectory passed a Choreo event marker",
                List.of(PortInfo.input("event", PortType.STRING, "Choreo event marker name")),
                n -> swerve.get().hasChoreoEventBeenPassed(n.getString("event")));
        condition(
                factory,
                "IsChassisStationary",
                "The chassis is not moving",
                List.of(),
                n -> swerve.get().isChassisStationary());
    }

    private static void condition(
            BehaviorTreeFactory factory,
            String id,
            String description,
            List<PortInfo> ports,
            java.util.function.Predicate<com.marswars.bt.core.TreeNode> predicate) {
        factory.registerLibraryNode(
                id,
                NodeKind.CONDITION,
                description,
                ports,
                (name, cfg) -> new SimpleConditionNode(name, cfg, predicate));
    }
}
