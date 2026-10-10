package com.marswars.bt.decorator;

import com.marswars.bt.core.DecoratorNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/**
 * Runs the child to completion once per tree instance. Afterwards returns SKIPPED ({@code
 * then_skip=true}, default) or keeps returning the stored result. BT.CPP {@code RunOnce}.
 */
public class RunOnceNode extends DecoratorNode {
    public static final String THEN_SKIP = "then_skip";

    private boolean already_ticked_ = false;
    private NodeStatus returned_status_ = NodeStatus.IDLE;

    public RunOnceNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        final boolean skip = getBoolean(THEN_SKIP, true);
        if (already_ticked_) {
            return skip ? NodeStatus.SKIPPED : returned_status_;
        }
        setStatus(NodeStatus.RUNNING);
        NodeStatus status = child().executeTick();
        if (status.isCompleted()) {
            already_ticked_ = true;
            returned_status_ = status;
            resetChild();
        }
        return status;
    }
}
