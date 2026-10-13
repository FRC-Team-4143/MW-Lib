package com.marswars.bt.control;

import com.marswars.bt.core.BtException;
import com.marswars.bt.core.ControlNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;
import java.util.HashSet;
import java.util.Set;

/**
 * Ticks every child each tick (completed children are not re-ticked). Succeeds once {@code
 * success_count} children succeeded, fails once {@code failure_count} failed or success became
 * impossible; either way the still-running children are halted. Negative thresholds count from the
 * number of children: {@code -1} means "all". BT.CPP v4 {@code Parallel}.
 *
 * <p>Ports: {@code success_count} (int, default -1), {@code failure_count} (int, default 1).
 */
public class ParallelNode extends ControlNode {
    public static final String SUCCESS_COUNT = "success_count";
    public static final String FAILURE_COUNT = "failure_count";

    private final Set<Integer> completed_list_ = new HashSet<>();
    private int success_threshold_ = -1;
    private int failure_threshold_ = 1;
    private int success_count_ = 0;
    private int failure_count_ = 0;

    public ParallelNode(String name, NodeConfig config) {
        super(name, config);
    }

    /** Hand-built constructor with explicit thresholds (ports are still read when present). */
    public ParallelNode(String name, NodeConfig config, int successCount, int failureCount) {
        super(name, config);
        success_threshold_ = successCount;
        failure_threshold_ = failureCount;
    }

    static int threshold(int value, int children) {
        return value < 0 ? Math.max(children + value + 1, 0) : value;
    }

    public int successThreshold() {
        return threshold(success_threshold_, childrenCount());
    }

    public int failureThreshold() {
        return threshold(failure_threshold_, childrenCount());
    }

    @Override
    protected NodeStatus tick() {
        success_threshold_ = getInt(SUCCESS_COUNT, success_threshold_);
        failure_threshold_ = getInt(FAILURE_COUNT, failure_threshold_);
        final int children_count = childrenCount();
        if (children_count < successThreshold()) {
            throw new BtException(
                    "[Parallel "
                            + getPath()
                            + "]: number of children is less than success_count. Can never succeed.");
        }
        if (children_count < failureThreshold()) {
            throw new BtException(
                    "[Parallel "
                            + getPath()
                            + "]: number of children is less than failure_count. Can never fail.");
        }

        setStatus(NodeStatus.RUNNING);
        int skipped_count = 0;

        for (int i = 0; i < children_count; i++) {
            if (!completed_list_.contains(i)) {
                NodeStatus child_status = child(i).executeTick();
                switch (child_status) {
                    case SKIPPED:
                        skipped_count++;
                        break;
                    case SUCCESS:
                        completed_list_.add(i);
                        success_count_++;
                        break;
                    case FAILURE:
                        completed_list_.add(i);
                        failure_count_++;
                        break;
                    case RUNNING:
                        break;
                    default:
                        throw new BtException("[" + getPath() + "]: child returned IDLE");
                }
            }

            final int required_success = successThreshold();
            if (success_count_ >= required_success
                    || (success_threshold_ < 0
                            && success_count_ + skipped_count >= required_success)) {
                clear();
                resetChildren();
                return NodeStatus.SUCCESS;
            }
            if ((children_count - failure_count_) < required_success
                    || failure_count_ == failureThreshold()) {
                clear();
                resetChildren();
                return NodeStatus.FAILURE;
            }
        }
        return skipped_count == children_count ? NodeStatus.SKIPPED : NodeStatus.RUNNING;
    }

    private void clear() {
        completed_list_.clear();
        success_count_ = 0;
        failure_count_ = 0;
    }

    @Override
    public void halt() {
        clear();
        super.halt();
    }
}
