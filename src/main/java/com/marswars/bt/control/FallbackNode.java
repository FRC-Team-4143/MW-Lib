package com.marswars.bt.control;

import com.marswars.bt.core.BtException;
import com.marswars.bt.core.ControlNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/**
 * Tries children in order until one succeeds. Returns RUNNING while a child runs (resuming there),
 * SUCCESS as soon as one succeeds, FAILURE when all fail. BT.CPP v4 {@code Fallback}.
 */
public class FallbackNode extends ControlNode {
    private int current_child_idx_ = 0;
    private int skipped_count_ = 0;

    public FallbackNode(String name, NodeConfig config) {
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
                case SUCCESS:
                    resetChildren();
                    current_child_idx_ = 0;
                    skipped_count_ = 0;
                    return NodeStatus.SUCCESS;
                case FAILURE:
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
        return all_skipped ? NodeStatus.SKIPPED : NodeStatus.FAILURE;
    }

    @Override
    public void halt() {
        current_child_idx_ = 0;
        skipped_count_ = 0;
        super.halt();
    }
}
