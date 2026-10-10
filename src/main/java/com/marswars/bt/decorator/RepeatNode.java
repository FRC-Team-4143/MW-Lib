package com.marswars.bt.decorator;

import com.marswars.bt.core.BtException;
import com.marswars.bt.core.DecoratorNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;

/**
 * Re-runs the child until it has succeeded {@code num_cycles} times ({@code -1} = forever); a child
 * FAILURE ends the loop with FAILURE. BT.CPP v4 {@code Repeat}.
 *
 * <p>Finite cycles of a synchronous child run within one tick, as in BT.CPP. With {@code
 * num_cycles=-1} this node yields RUNNING after every completed cycle instead, so a synchronous
 * child can never spin the robot loop forever inside a single tick.
 */
public class RepeatNode extends DecoratorNode {
    public static final String NUM_CYCLES = "num_cycles";

    private int repeat_count_ = 0;

    public RepeatNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus tick() {
        final int num_cycles = getInt(NUM_CYCLES);
        boolean do_loop = repeat_count_ < num_cycles || num_cycles == -1;
        setStatus(NodeStatus.RUNNING);

        while (do_loop) {
            NodeStatus child_status = child().executeTick();
            switch (child_status) {
                case SUCCESS:
                    repeat_count_++;
                    do_loop = repeat_count_ < num_cycles || num_cycles == -1;
                    resetChild();
                    if (num_cycles == -1) {
                        return NodeStatus.RUNNING; // yield: never loop forever in one tick
                    }
                    break;
                case FAILURE:
                    repeat_count_ = 0;
                    resetChild();
                    return NodeStatus.FAILURE;
                case RUNNING:
                    return NodeStatus.RUNNING;
                case SKIPPED:
                    resetChild();
                    return NodeStatus.SKIPPED;
                default:
                    throw new BtException("[" + getPath() + "]: child returned IDLE");
            }
        }
        repeat_count_ = 0;
        return NodeStatus.SUCCESS;
    }

    @Override
    public void halt() {
        repeat_count_ = 0;
        super.halt();
    }
}
