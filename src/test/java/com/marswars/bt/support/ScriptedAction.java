package com.marswars.bt.support;

import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.StatefulActionNode;
import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Stateful action whose results are scripted: each start/running call pops the next status from
 * the queue (repeating the last when the queue is empty). Counts lifecycle calls.
 */
public final class ScriptedAction extends StatefulActionNode {
    private final Deque<NodeStatus> script_ = new ArrayDeque<>();
    private NodeStatus last_ = NodeStatus.SUCCESS;
    public int starts = 0;
    public int runs = 0;
    public int halts = 0;

    public ScriptedAction(String name, NodeConfig config, NodeStatus... script) {
        super(name, config);
        script(script);
    }

    public ScriptedAction script(NodeStatus... statuses) {
        script_.clear();
        for (NodeStatus s : statuses) {
            script_.add(s);
        }
        if (statuses.length > 0) {
            last_ = statuses[statuses.length - 1];
        }
        return this;
    }

    private NodeStatus next() {
        return script_.isEmpty() ? last_ : script_.poll();
    }

    @Override
    protected NodeStatus onStart() {
        starts++;
        return next();
    }

    @Override
    protected NodeStatus onRunning() {
        runs++;
        return next();
    }

    @Override
    protected void onHalted() {
        halts++;
    }

    /** Total start + running ticks. */
    public int ticks() {
        return starts + runs;
    }
}
