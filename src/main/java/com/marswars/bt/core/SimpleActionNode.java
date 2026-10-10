package com.marswars.bt.core;

import java.util.Objects;
import java.util.function.Function;

/** A synchronous action backed by a function (BT.CPP {@code SimpleActionNode}). */
public class SimpleActionNode extends SyncActionNode {
    private final Function<TreeNode, NodeStatus> function_;

    public SimpleActionNode(String name, NodeConfig config, Function<TreeNode, NodeStatus> function) {
        super(name, config);
        function_ = Objects.requireNonNull(function, "function");
    }

    @Override
    protected NodeStatus tick() {
        return function_.apply(this);
    }
}
