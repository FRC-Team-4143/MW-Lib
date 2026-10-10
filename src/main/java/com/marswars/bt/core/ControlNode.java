package com.marswars.bt.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** A node with an ordered list of children (sequences, fallbacks, parallels, branches). */
public abstract class ControlNode extends TreeNode {
    private final List<TreeNode> children_ = new ArrayList<>();

    protected ControlNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    public NodeKind kind() {
        return NodeKind.CONTROL;
    }

    public void addChild(TreeNode child) {
        children_.add(child);
    }

    public final List<TreeNode> children() {
        return Collections.unmodifiableList(children_);
    }

    @Override
    public final List<TreeNode> childNodes() {
        return children();
    }

    public final TreeNode child(int index) {
        return children_.get(index);
    }

    public final int childrenCount() {
        return children_.size();
    }

    /** Halts child {@code index} if it is RUNNING, then resets its status to IDLE. */
    public final void haltChild(int index) {
        TreeNode child = children_.get(index);
        if (child.getStatus() == NodeStatus.RUNNING) {
            child.haltNode();
        }
        child.resetStatus();
    }

    /** Halts/resets every child starting at {@code from}. */
    public final void haltChildren(int from) {
        for (int i = from; i < children_.size(); i++) {
            haltChild(i);
        }
    }

    public final void haltChildren() {
        haltChildren(0);
    }

    /** Same as {@link #haltChildren()}: every child ends IDLE. */
    public final void resetChildren() {
        haltChildren(0);
    }

    @Override
    public void halt() {
        resetChildren();
        resetStatus();
    }
}
