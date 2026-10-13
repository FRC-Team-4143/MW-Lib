package com.marswars.bt.control;

import com.marswars.bt.core.BtException;
import com.marswars.bt.core.ControlNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;
import java.util.HashSet;
import java.util.Set;

/**
 * Runs every child in parallel until the <b>first child</b> (the deadline) finishes, then halts
 * the others and returns the deadline's result. The other children's results are ignored; one
 * that finishes early is simply not ticked again. Same idea as WPILib's {@code
 * Commands.deadline}. MW-Lib node (not part of BT.CPP).
 *
 * <pre>{@code
 * <ParallelDeadline>
 *   <FollowTrajectory trajectory="SynergyP1"/>          <!-- decides when this ends -->
 *   <SubTree ID="IntakeOnEvent" event="Intake Out"/>   <!-- runs alongside -->
 * </ParallelDeadline>
 * }</pre>
 */
public class ParallelDeadlineNode extends ControlNode {
    private final Set<Integer> completed_ = new HashSet<>();

    public ParallelDeadlineNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        if (childrenCount() == 0) {
            throw new BtException("[ParallelDeadline " + getPath() + "] needs at least one child");
        }
        setStatus(NodeStatus.RUNNING);

        NodeStatus deadline = NodeStatus.RUNNING;
        for (int i = 0; i < childrenCount(); i++) {
            if (completed_.contains(i)) {
                continue;
            }
            NodeStatus s = child(i).executeTick();
            if (i == 0) {
                deadline = s;
            } else if (s.isCompleted() || s == NodeStatus.SKIPPED) {
                completed_.add(i);
            }
        }

        if (deadline == NodeStatus.RUNNING) {
            return NodeStatus.RUNNING;
        }
        completed_.clear();
        resetChildren(); // halts the children still running
        return deadline;
    }

    @Override
    public void halt() {
        completed_.clear();
        super.halt();
    }
}
