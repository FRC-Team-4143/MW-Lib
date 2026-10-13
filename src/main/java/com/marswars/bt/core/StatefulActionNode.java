package com.marswars.bt.core;

/**
 * An action that may take several ticks. Mirrors BT.CPP {@code StatefulActionNode}: the first tick
 * calls {@link #onStart()}, later ticks call {@link #onRunning()} while RUNNING, and a halt while
 * RUNNING calls {@link #onHalted()}. A node whose {@code onStart()} never ran is never told to halt.
 * Once completed, the node keeps returning the same status until it is reset by its parent.
 */
public abstract class StatefulActionNode extends LeafNode {
    protected StatefulActionNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    public NodeKind kind() {
        return NodeKind.ACTION;
    }

    @Override
    protected final NodeStatus tick() {
        NodeStatus previous = getStatus();
        if (previous == NodeStatus.IDLE) {
            setStatus(NodeStatus.RUNNING);
            NodeStatus result = onStart();
            if (result == null || result == NodeStatus.IDLE) {
                throw new BtException("[" + getPath() + "] onStart() must not return IDLE");
            }
            return result;
        }
        if (previous == NodeStatus.RUNNING) {
            NodeStatus result = onRunning();
            if (result == null || result == NodeStatus.IDLE) {
                throw new BtException("[" + getPath() + "] onRunning() must not return IDLE");
            }
            return result;
        }
        return previous;
    }

    @Override
    public final void halt() {
        if (getStatus() == NodeStatus.RUNNING) {
            onHalted();
        }
        resetStatus();
    }

    /** Called on the first tick; return RUNNING to be ticked again, or a final status. */
    protected abstract NodeStatus onStart();

    /** Called on every tick after {@link #onStart()} returned RUNNING. */
    protected abstract NodeStatus onRunning();

    /** Called when a parent halts this node while it is RUNNING. */
    protected abstract void onHalted();
}
