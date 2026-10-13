package com.marswars.bt.decorator;

import com.marswars.bt.core.DecoratorNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/**
 * Restarts the child every time it succeeds and stays RUNNING; returns FAILURE when the child
 * fails. With an {@code AlwaysSuccess} child it simply runs forever, which is the idiom for "park
 * this branch until the parent halts it". BT.CPP {@code KeepRunningUntilFailure}.
 */
public class KeepRunningUntilFailureNode extends DecoratorNode {
    public KeepRunningUntilFailureNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        setStatus(NodeStatus.RUNNING);
        NodeStatus child_status = child().executeTick();
        switch (child_status) {
            case FAILURE:
                resetChild();
                return NodeStatus.FAILURE;
            case SUCCESS:
                resetChild();
                return NodeStatus.RUNNING;
            case RUNNING:
                return NodeStatus.RUNNING;
            default:
                return getStatus();
        }
    }
}
