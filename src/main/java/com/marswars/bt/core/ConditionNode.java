package com.marswars.bt.core;

/** A leaf that answers SUCCESS or FAILURE immediately and never RUNNING. */
public abstract class ConditionNode extends LeafNode {
    protected ConditionNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    public NodeKind kind() {
        return NodeKind.CONDITION;
    }

    @Override
    protected final void validateTickResult(NodeStatus result) {
        if (result == NodeStatus.RUNNING) {
            throw new BtException("ConditionNode [" + getPath() + "] must not return RUNNING");
        }
    }
}
