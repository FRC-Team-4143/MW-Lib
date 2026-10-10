package com.marswars.bt.core;

import java.util.List;

/** A node with exactly one child whose result it transforms, repeats, times or gates. */
public abstract class DecoratorNode extends TreeNode {
    private TreeNode child_ = null;

    protected DecoratorNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    public NodeKind kind() {
        return NodeKind.DECORATOR;
    }

    public void setChild(TreeNode child) {
        child_ = child;
    }

    public final TreeNode child() {
        if (child_ == null) {
            throw new BtException("Decorator [" + getPath() + "] has no child");
        }
        return child_;
    }

    public final boolean hasChild() {
        return child_ != null;
    }

    @Override
    public final List<TreeNode> childNodes() {
        return child_ == null ? List.of() : List.of(child_);
    }

    /** Halts the child if RUNNING, then resets it to IDLE. */
    public final void haltChild() {
        if (child_ == null) {
            return;
        }
        if (child_.getStatus() == NodeStatus.RUNNING) {
            child_.haltNode();
        }
        child_.resetStatus();
    }

    public final void resetChild() {
        haltChild();
    }

    @Override
    public void halt() {
        resetChild();
        resetStatus();
    }
}
