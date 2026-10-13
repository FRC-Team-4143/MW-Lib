package com.marswars.bt.control;

import com.marswars.bt.core.BtException;
import com.marswars.bt.core.ControlNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/**
 * {@code if (child0) child1 else child2}. The condition is evaluated once; the chosen branch then
 * runs to completion without re-checking it. With only two children a failed condition returns
 * FAILURE. BT.CPP v4 {@code IfThenElse}.
 */
public class IfThenElseNode extends ControlNode {
    private int child_idx_ = 0;

    public IfThenElseNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        final int children_count = childrenCount();
        if (children_count != 2 && children_count != 3) {
            throw new BtException(
                    "[IfThenElse " + getPath() + "] must have either 2 or 3 children");
        }
        setStatus(NodeStatus.RUNNING);

        if (child_idx_ == 0) {
            NodeStatus condition_status = child(0).executeTick();
            if (condition_status == NodeStatus.RUNNING) {
                return condition_status;
            } else if (condition_status == NodeStatus.SUCCESS) {
                child_idx_ = 1;
            } else if (condition_status == NodeStatus.FAILURE) {
                if (children_count == 3) {
                    child_idx_ = 2;
                } else {
                    return condition_status;
                }
            } else {
                throw new BtException(
                        "[IfThenElse " + getPath() + "]: condition returned " + condition_status);
            }
        }

        NodeStatus status = child(child_idx_).executeTick();
        if (status == NodeStatus.RUNNING) {
            return NodeStatus.RUNNING;
        }
        resetChildren();
        child_idx_ = 0;
        return status;
    }

    @Override
    public void halt() {
        child_idx_ = 0;
        super.halt();
    }
}
