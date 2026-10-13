package com.marswars.bt.monitor;

import com.google.gson.JsonArray;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;
import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.TreeNode;
import com.marswars.bt.decorator.SubTreeNode;

/**
 * JSON description of an instantiated tree's nodes in depth-first pre-order, in the shape of the
 * btlive {@code tree} message: {@code uid, parent, type, name, category, path[, subtree]}.
 */
public final class TreeStructure {
    private TreeStructure() {}

    public static JsonArray nodes(BehaviorTree tree) {
        JsonArray nodes = new JsonArray();
        for (TreeNode node : tree.getNodes()) {
            JsonObject n = new JsonObject();
            n.addProperty("uid", node.getUid());
            TreeNode parent = tree.getParent(node).orElse(null);
            if (parent == null) {
                n.add("parent", JsonNull.INSTANCE);
            } else {
                n.addProperty("parent", parent.getUid());
            }
            n.addProperty("type", node.getRegistrationId());
            n.addProperty("name", node.getName());
            n.addProperty("category", node.kind().xmlTag());
            n.addProperty("path", node.getPath());
            if (node instanceof SubTreeNode subtree) {
                n.addProperty("subtree", subtree.subtreeId());
            }
            nodes.add(n);
        }
        return nodes;
    }

    /** {@code {"tree_id": ..., "nodes": [...]}}. */
    public static JsonObject describe(BehaviorTree tree) {
        JsonObject o = new JsonObject();
        o.addProperty("tree_id", tree.getMainTreeId());
        o.add("nodes", nodes(tree));
        return o;
    }
}
