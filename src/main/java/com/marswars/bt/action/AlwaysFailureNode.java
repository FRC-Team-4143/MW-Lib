package com.marswars.bt.action;

import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.SyncActionNode;

/** Always returns FAILURE. BT.CPP {@code AlwaysFailure}. */
public class AlwaysFailureNode extends SyncActionNode {
    public AlwaysFailureNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        return NodeStatus.FAILURE;
    }
}
