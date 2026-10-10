package com.marswars.bt.control;

import com.marswars.bt.core.BtException;
import com.marswars.bt.core.ControlNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/**
 * A fallback that re-evaluates every child from the first one on every tick: if an earlier child
 * starts succeeding, the running later child is halted. BT.CPP v4 {@code ReactiveFallback}.
 */
public class ReactiveFallbackNode extends ControlNode {
    private static boolean throw_if_multiple_running_ = false;

    private int running_child_ = -1;

    public ReactiveFallbackNode(String name, NodeConfig config) {
        super(name, config);
    }

    /** Throw when two different children return RUNNING (BT.CPP {@code EnableException}). */
    public static void enableException(boolean enable) {
        throw_if_multiple_running_ = enable;
    }

    @Override
    protected NodeStatus tick() {
        boolean all_skipped = true;
        if (getStatus() == NodeStatus.IDLE) {
            running_child_ = -1;
        }
        setStatus(NodeStatus.RUNNING);

        for (int index = 0; index < childrenCount(); index++) {
            NodeStatus child_status = child(index).executeTick();
            all_skipped &= child_status == NodeStatus.SKIPPED;
            switch (child_status) {
                case RUNNING:
                    for (int i = 0; i < childrenCount(); i++) {
                        if (i != index) {
                            haltChild(i);
                        }
                    }
                    if (running_child_ == -1) {
                        running_child_ = index;
                    } else if (throw_if_multiple_running_ && running_child_ != index) {
                        throw new BtException(
                                "[ReactiveFallback "
                                        + getPath()
                                        + "]: only a single child can return RUNNING");
                    }
                    return NodeStatus.RUNNING;
                case FAILURE:
                    break;
                case SUCCESS:
                    resetChildren();
                    return NodeStatus.SUCCESS;
                case SKIPPED:
                    haltChild(index);
                    break;
                default:
                    throw new BtException("[" + getPath() + "]: child returned IDLE");
            }
        }
        resetChildren();
        return all_skipped ? NodeStatus.SKIPPED : NodeStatus.FAILURE;
    }

    @Override
    public void halt() {
        running_child_ = -1;
        super.halt();
    }
}
