package com.marswars.bt.core;

/** A node without children. */
public abstract class LeafNode extends TreeNode {
    protected LeafNode(String name, NodeConfig config) {
        super(name, config);
    }
}
