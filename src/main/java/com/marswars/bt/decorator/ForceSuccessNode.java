package com.marswars.bt.decorator;

import com.marswars.bt.core.DecoratorNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/** Returns SUCCESS once the child completes; RUNNING/SKIPPED pass through. BT.CPP {@code ForceSuccess}. */
public class ForceSuccessNode extends DecoratorNode {
    public ForceSuccessNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        setStatus(NodeStatus.RUNNING);
        NodeStatus child_status = child().executeTick();
        if (child_status.isCompleted()) {
            resetChild();
            return NodeStatus.SUCCESS;
        }
        return child_status;
    }
}
