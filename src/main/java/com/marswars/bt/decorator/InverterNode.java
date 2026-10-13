package com.marswars.bt.decorator;

import com.marswars.bt.core.DecoratorNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/** Swaps the child's SUCCESS and FAILURE; RUNNING/SKIPPED pass through. BT.CPP {@code Inverter}. */
public class InverterNode extends DecoratorNode {
    public InverterNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        setStatus(NodeStatus.RUNNING);
        NodeStatus child_status = child().executeTick();
        switch (child_status) {
            case SUCCESS:
                resetChild();
                return NodeStatus.FAILURE;
            case FAILURE:
                resetChild();
                return NodeStatus.SUCCESS;
            default:
                return child_status;
        }
    }
}
