package com.marswars.bt.action;

import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.SyncActionNode;

/** Always returns SUCCESS. BT.CPP {@code AlwaysSuccess}. */
public class AlwaysSuccessNode extends SyncActionNode {
    public AlwaysSuccessNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        return NodeStatus.SUCCESS;
    }
}
