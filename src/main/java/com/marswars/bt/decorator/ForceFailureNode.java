package com.marswars.bt.decorator;

import com.marswars.bt.core.DecoratorNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/** Returns FAILURE once the child completes; RUNNING/SKIPPED pass through. BT.CPP {@code ForceFailure}. */
public class ForceFailureNode extends DecoratorNode {
    public ForceFailureNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        setStatus(NodeStatus.RUNNING);
        NodeStatus child_status = child().executeTick();
        if (child_status.isCompleted()) {
            resetChild();
            return NodeStatus.FAILURE;
        }
        return child_status;
    }
}
