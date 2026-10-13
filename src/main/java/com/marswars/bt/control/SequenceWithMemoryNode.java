package com.marswars.bt.control;

import com.marswars.bt.core.BtException;
import com.marswars.bt.core.ControlNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/**
 * Like {@link SequenceNode}, but a FAILURE does not rewind: the next tick resumes at the failed
 * child (already-succeeded children are not re-run). The index also survives a halt. BT.CPP v4
 * {@code SequenceWithMemory} (formerly {@code SequenceStar}).
 */
public class SequenceWithMemoryNode extends ControlNode {
    private int current_child_idx_ = 0;
    private int skipped_count_ = 0;

    public SequenceWithMemoryNode(String name, NodeConfig config) {
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
                    // Deliberately keep current_child_idx_: resume here next time.
                    haltChildren(current_child_idx_);
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

    /** Index of the child the next tick resumes at (for tests/monitoring). */
    public int currentChildIndex() {
        return current_child_idx_;
    }

    @Override
    public void halt() {
        // BT.CPP keeps current_child_idx_ across halts for SequenceWithMemory.
        super.halt();
    }
}
