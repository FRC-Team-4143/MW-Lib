package com.marswars.bt.support;

import com.marswars.bt.core.ConditionNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/** Condition with a settable result that counts its ticks. */
public final class Flag extends ConditionNode {
    public boolean value;
    public int ticks = 0;

    public Flag(String name, NodeConfig config, boolean initial) {
        super(name, config);
        value = initial;
    }

    @Override
    protected NodeStatus tick() {
        ticks++;
        return value ? NodeStatus.SUCCESS : NodeStatus.FAILURE;
    }
}
