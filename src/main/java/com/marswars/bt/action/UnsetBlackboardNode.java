package com.marswars.bt.action;

import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.PortValues;
import com.marswars.bt.core.SyncActionNode;

/** Removes the blackboard entry named by {@code key}. BT.CPP {@code UnsetBlackboard}. */
public class UnsetBlackboardNode extends SyncActionNode {
    public static final String KEY = "key";

    public UnsetBlackboardNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        String key = getRawInput(KEY).orElse("");
        if (PortValues.isPointer(key)) {
            key = PortValues.pointerKey(key);
        }
        blackboard().set(key, null);
        return NodeStatus.SUCCESS;
    }
}
