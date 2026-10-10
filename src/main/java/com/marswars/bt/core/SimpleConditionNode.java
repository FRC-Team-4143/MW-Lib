package com.marswars.bt.core;

import java.util.Objects;
import java.util.function.Predicate;

/** A condition backed by a predicate (BT.CPP {@code SimpleConditionNode}). */
public class SimpleConditionNode extends ConditionNode {
    private final Predicate<TreeNode> predicate_;

    public SimpleConditionNode(String name, NodeConfig config, Predicate<TreeNode> predicate) {
        super(name, config);
        predicate_ = Objects.requireNonNull(predicate, "predicate");
    }

    @Override
    protected NodeStatus tick() {
        return predicate_.test(this) ? NodeStatus.SUCCESS : NodeStatus.FAILURE;
    }
}
