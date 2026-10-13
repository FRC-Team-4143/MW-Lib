package com.marswars.bt.decorator;

import com.marswars.bt.core.DecoratorNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/**
 * Returns RUNNING for {@code delay_msec} milliseconds, then ticks the child and returns its result.
 * Re-arms after the child completes or the node is halted. Uses the tree clock. BT.CPP {@code
 * Delay}.
 */
public class DelayNode extends DecoratorNode {
    public static final String DELAY_MSEC = "delay_msec";

    private boolean delay_started_ = false;
    private double start_time_ = 0.0;

    public DelayNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        final double msec = getDouble(DELAY_MSEC);
        if (!delay_started_) {
            delay_started_ = true;
            start_time_ = now();
            setStatus(NodeStatus.RUNNING);
        }

        if ((now() - start_time_) * 1000.0 < msec) {
            return NodeStatus.RUNNING;
        }
        NodeStatus child_status = child().executeTick();
        if (child_status.isCompleted()) {
            delay_started_ = false;
            resetChild();
        }
        return child_status;
    }

    @Override
    public void halt() {
        delay_started_ = false;
        super.halt();
    }
}
