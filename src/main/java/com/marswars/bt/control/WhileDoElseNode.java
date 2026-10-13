package com.marswars.bt.control;

import com.marswars.bt.core.BtException;
import com.marswars.bt.core.ControlNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/**
 * Reactive {@code if}: the condition (child 0) is re-ticked every tick. SUCCESS runs child 1 and
 * halts child 2; FAILURE runs child 2 (or returns FAILURE with only two children) and halts child 1.
 * BT.CPP v4 {@code WhileDoElse}.
 */
public class WhileDoElseNode extends ControlNode {
    public WhileDoElseNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        final int children_count = childrenCount();
        if (children_count != 2 && children_count != 3) {
            throw new BtException(
                    "[WhileDoElse " + getPath() + "] must have either 2 or 3 children");
        }
        setStatus(NodeStatus.RUNNING);

        NodeStatus condition_status = child(0).executeTick();
        if (condition_status == NodeStatus.RUNNING) {
            return condition_status;
        }

        NodeStatus status;
        if (condition_status == NodeStatus.SUCCESS) {
            if (children_count == 3) {
                haltChild(2);
            }
            status = child(1).executeTick();
        } else if (condition_status == NodeStatus.FAILURE) {
            if (children_count == 3) {
                haltChild(1);
                status = child(2).executeTick();
            } else {
                status = NodeStatus.FAILURE;
            }
        } else {
            throw new BtException(
                    "[WhileDoElse " + getPath() + "]: condition returned " + condition_status);
        }

        if (status == NodeStatus.RUNNING) {
            return NodeStatus.RUNNING;
        }
        resetChildren();
        return status;
    }
}
