package com.marswars.bt.control;

import com.marswars.bt.core.BtException;
import com.marswars.bt.core.ControlNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;
import java.util.HashSet;
import java.util.Set;

/**
 * Runs every child to completion (no early exit). Returns FAILURE if at least {@code max_failures}
 * children failed, SUCCESS otherwise. BT.CPP v4 {@code ParallelAll}.
 *
 * <p>Port: {@code max_failures} (int, default 1; negative counts from the number of children).
 */
public class ParallelAllNode extends ControlNode {
    public static final String MAX_FAILURES = "max_failures";

    private final Set<Integer> completed_list_ = new HashSet<>();
    private int failure_count_ = 0;

    public ParallelAllNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        final int children_count = childrenCount();
        final int failure_threshold =
                ParallelNode.threshold(getInt(MAX_FAILURES, 1), children_count);
        if (children_count < failure_threshold) {
            throw new BtException(
                    "[ParallelAll "
                            + getPath()
                            + "]: number of children is less than max_failures. Can never fail.");
        }

        setStatus(NodeStatus.RUNNING);
        int skipped_count = 0;

        for (int i = 0; i < children_count; i++) {
            if (completed_list_.contains(i)) {
                continue;
            }
            NodeStatus child_status = child(i).executeTick();
            switch (child_status) {
                case SUCCESS:
                    completed_list_.add(i);
                    break;
                case FAILURE:
                    completed_list_.add(i);
                    failure_count_++;
                    break;
                case RUNNING:
                    break;
                case SKIPPED:
                    skipped_count++;
                    break;
                default:
                    throw new BtException("[" + getPath() + "]: child returned IDLE");
            }
        }

        if (skipped_count == children_count) {
            return NodeStatus.SKIPPED;
        }
        if (skipped_count + completed_list_.size() >= children_count) {
            haltChildren();
            completed_list_.clear();
            NodeStatus result =
                    failure_count_ >= failure_threshold ? NodeStatus.FAILURE : NodeStatus.SUCCESS;
            failure_count_ = 0;
            return result;
        }
        return NodeStatus.RUNNING;
    }

    @Override
    public void halt() {
        completed_list_.clear();
        failure_count_ = 0;
        super.halt();
    }
}
