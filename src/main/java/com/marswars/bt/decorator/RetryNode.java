package com.marswars.bt.decorator;

import com.marswars.bt.core.BtException;
import com.marswars.bt.core.DecoratorNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/**
 * Re-runs a failing child up to {@code num_attempts} times ({@code -1} = forever); returns SUCCESS
 * as soon as it succeeds. BT.CPP v4 {@code RetryUntilSuccessful}.
 *
 * <p>With {@code num_attempts=-1} this node yields RUNNING after every failed attempt (see {@link
 * RepeatNode} for why).
 */
public class RetryNode extends DecoratorNode {
    public static final String NUM_ATTEMPTS = "num_attempts";

    private int try_count_ = 0;
    private boolean all_skipped_ = true;

    public RetryNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected String defaultRegistrationId() {
        return "RetryUntilSuccessful";
    }

    @Override
    protected NodeStatus tick() {
        final int max_attempts = getInt(NUM_ATTEMPTS);
        boolean do_loop = try_count_ < max_attempts || max_attempts == -1;
        if (getStatus() == NodeStatus.IDLE) {
            all_skipped_ = true;
        }
        setStatus(NodeStatus.RUNNING);

        while (do_loop) {
            NodeStatus child_status = child().executeTick();
            all_skipped_ &= child_status == NodeStatus.SKIPPED;
            switch (child_status) {
                case SUCCESS:
                    try_count_ = 0;
                    resetChild();
                    return NodeStatus.SUCCESS;
                case FAILURE:
                    try_count_++;
                    do_loop = try_count_ < max_attempts || max_attempts == -1;
                    resetChild();
                    if (max_attempts == -1) {
                        return NodeStatus.RUNNING; // yield: never loop forever in one tick
                    }
                    break;
                case RUNNING:
                    return NodeStatus.RUNNING;
                case SKIPPED:
                    resetChild();
                    return NodeStatus.SKIPPED;
                default:
                    throw new BtException("[" + getPath() + "]: child returned IDLE");
            }
        }
        try_count_ = 0;
        return all_skipped_ ? NodeStatus.SKIPPED : NodeStatus.FAILURE;
    }

    @Override
    public void halt() {
        try_count_ = 0;
        super.halt();
    }
}
