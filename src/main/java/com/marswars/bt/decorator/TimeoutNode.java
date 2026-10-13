package com.marswars.bt.decorator;

import com.marswars.bt.core.DecoratorNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/**
 * Halts the child and returns FAILURE if it is still RUNNING {@code msec} milliseconds after this
 * node started; otherwise returns the child's result. {@code msec <= 0} disables the timeout. Uses
 * the tree clock (no timer thread). BT.CPP {@code Timeout}.
 */
public class TimeoutNode extends DecoratorNode {
    public static final String MSEC = "msec";

    private boolean timeout_started_ = false;
    private double start_time_ = 0.0;

    public TimeoutNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        final double msec = getDouble(MSEC);
        if (!timeout_started_) {
            timeout_started_ = true;
            start_time_ = now();
            setStatus(NodeStatus.RUNNING);
        }

        if (msec > 0
                && child().getStatus() == NodeStatus.RUNNING
                && (now() - start_time_) * 1000.0 >= msec) {
            haltChild();
            timeout_started_ = false;
            return NodeStatus.FAILURE;
        }

        NodeStatus child_status = child().executeTick();
        if (child_status.isCompleted()) {
            timeout_started_ = false;
            resetChild();
        }
        return child_status;
    }

    @Override
    public void halt() {
        timeout_started_ = false;
        super.halt();
    }
}
