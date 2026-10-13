package com.marswars.bt.core;

/** An action that completes within a single tick: {@link #tick()} may not return RUNNING. */
public abstract class SyncActionNode extends LeafNode {
    protected SyncActionNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    public NodeKind kind() {
        return NodeKind.ACTION;
    }

    @Override
    protected final void validateTickResult(NodeStatus result) {
        if (result == NodeStatus.RUNNING) {
            throw new BtException("SyncActionNode [" + getPath() + "] must not return RUNNING");
        }
    }
}
