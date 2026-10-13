package com.marswars.bt.action;

import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.StatefulActionNode;

/**
 * Base for the waiting leaves: RUNNING until {@link #durationSeconds()} (read at start) has passed
 * on the tree clock, then SUCCESS. A non-positive duration succeeds on the first tick.
 */
public abstract class TimedWaitNode extends StatefulActionNode {
    private double end_time_ = 0.0;

    protected TimedWaitNode(String name, NodeConfig config) {
        super(name, config);
    }

    /** Wait length in seconds; evaluated once, when the node starts. */
    protected abstract double durationSeconds();

    @Override
    protected NodeStatus onStart() {
        double duration = durationSeconds();
        if (duration <= 0.0) {
            return NodeStatus.SUCCESS;
        }
        end_time_ = now() + duration;
        return NodeStatus.RUNNING;
    }

    @Override
    protected NodeStatus onRunning() {
        return now() >= end_time_ ? NodeStatus.SUCCESS : NodeStatus.RUNNING;
    }

    @Override
    protected void onHalted() {}
}
