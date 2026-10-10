package com.marswars.bt.control;

import com.marswars.bt.core.BtException;
import com.marswars.bt.core.ControlNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/**
 * A sequence that re-evaluates every child from the first one on every tick, so earlier conditions
 * keep guarding a running action. When child {@code i} is RUNNING, every other child is halted. A
 * later FAILURE of an earlier child halts the running one. BT.CPP v4 {@code ReactiveSequence}.
 *
 * <p>Only one asynchronous child should be able to return RUNNING. As in BT.CPP, a second RUNNING
 * child is tolerated (the other one is halted) unless {@link #enableException(boolean)} is on.
 */
public class ReactiveSequenceNode extends ControlNode {
    private static boolean throw_if_multiple_running_ = false;

    private int running_child_ = -1;

    public ReactiveSequenceNode(String name, NodeConfig config) {
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
                    // Reset the other children so they are re-ticked from IDLE next time.
                    for (int i = 0; i < childrenCount(); i++) {
                        if (i != index) {
                            haltChild(i);
                        }
                    }
                    if (running_child_ == -1) {
                        running_child_ = index;
                    } else if (throw_if_multiple_running_ && running_child_ != index) {
                        throw new BtException(
                                "[ReactiveSequence "
                                        + getPath()
                                        + "]: only a single child can return RUNNING");
                    }
                    return NodeStatus.RUNNING;
                case FAILURE:
                    resetChildren();
                    return NodeStatus.FAILURE;
                case SUCCESS:
                    break;
                case SKIPPED:
                    haltChild(index);
                    break;
                default:
                    throw new BtException("[" + getPath() + "]: child returned IDLE");
            }
        }
        resetChildren();
        return all_skipped ? NodeStatus.SKIPPED : NodeStatus.SUCCESS;
    }

    @Override
    public void halt() {
        running_child_ = -1;
        super.halt();
    }
}
