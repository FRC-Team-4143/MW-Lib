package com.marswars.bt.control;

import com.marswars.bt.core.BtException;
import com.marswars.bt.core.ControlNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/**
 * Ticks children in order. Returns RUNNING while a child runs (resuming at that child next tick),
 * FAILURE as soon as one fails (and restarts from the first child), SUCCESS when all succeed.
 * BT.CPP v4 {@code Sequence}.
 */
public class SequenceNode extends ControlNode {
    private int current_child_idx_ = 0;
    private int skipped_count_ = 0;

    public SequenceNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        final int count = childrenCount();
        if (getStatus() == NodeStatus.IDLE) {
            skipped_count_ = 0;
        }
        setStatus(NodeStatus.RUNNING);

        while (current_child_idx_ < count) {
            NodeStatus child_status = child(current_child_idx_).executeTick();
            switch (child_status) {
                case RUNNING:
                    return NodeStatus.RUNNING;
                case FAILURE:
                    resetChildren();
                    current_child_idx_ = 0;
                    skipped_count_ = 0;
                    return NodeStatus.FAILURE;
                case SUCCESS:
                    current_child_idx_++;
                    break;
                case SKIPPED:
                    current_child_idx_++;
                    skipped_count_++;
                    break;
                default:
                    throw new BtException("[" + getPath() + "]: child returned IDLE");
            }
        }
        boolean all_skipped = skipped_count_ == count;
        resetChildren();
        current_child_idx_ = 0;
        skipped_count_ = 0;
        return all_skipped ? NodeStatus.SKIPPED : NodeStatus.SUCCESS;
    }

    @Override
    public void halt() {
        current_child_idx_ = 0;
        skipped_count_ = 0;
        super.halt();
    }
}
