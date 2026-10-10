package com.marswars.bt.action;

import com.marswars.bt.core.BtException;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.PortValues;
import com.marswars.bt.core.SyncActionNode;

/**
 * Writes {@code value} into the blackboard entry named by {@code output_key}. A {@code {key}} value
 * copies another entry; anything else is stored as the literal string. BT.CPP {@code SetBlackboard}.
 */
public class SetBlackboardNode extends SyncActionNode {
    public static final String VALUE = "value";
    public static final String OUTPUT_KEY = "output_key";

    public SetBlackboardNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        String key =
                getRawInput(OUTPUT_KEY)
                        .orElseThrow(() -> new BtException("[" + getPath() + "] missing output_key"));
        if (PortValues.isPointer(key)) {
            key = PortValues.pointerKey(key);
        }
        String raw =
                getRawInput(VALUE)
                        .orElseThrow(() -> new BtException("[" + getPath() + "] missing value"));
        Object value = PortValues.isPointer(raw) ? blackboard().get(PortValues.pointerKey(raw, VALUE)) : raw;
        blackboard().set(key, value);
        return NodeStatus.SUCCESS;
    }
}
