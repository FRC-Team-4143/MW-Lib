package com.marswars.bt.core;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Fluent, code-only way to write the same trees as BT.CPP XML. Port values are given as
 * key/value pairs exactly like XML attributes; the pseudo-port {@code "name"} sets the instance
 * name.
 *
 * <pre>{@code
 * TreeSpec spec = TreeBuilder.create("MainTree")
 *     .begin("Sequence", "name", "Auto")
 *         .node("SetIntakeState", "state", "STORE")
 *         .begin("Parallel", "success_count", "1")
 *             .node("FollowTrajectory", "trajectory", "SynergyP1")
 *             .node("WaitForChoreoEvent", "event", "Intake Out")
 *         .end()
 *         .node("Wait", "seconds", "2")
 *     .end()
 *     .build();
 * BehaviorTree tree = factory.createTree(spec);
 * }</pre>
 */
public final class TreeBuilder {
    private static final class Frame {
        final String id;
        final String name;
        final Map<String, String> attributes;
        final String subtree_id;
        final List<NodeSpec> children = new ArrayList<>();

        Frame(String id, String name, Map<String, String> attributes, String subtreeId) {
            this.id = id;
            this.name = name;
            this.attributes = attributes;
            this.subtree_id = subtreeId;
        }

        NodeSpec toSpec() {
            return new NodeSpec(id, name, attributes, children, subtree_id, 0);
        }
    }

    private final String main_tree_id_;
    private final Map<String, NodeSpec> trees_ = new LinkedHashMap<>();
    private final Deque<Frame> stack_ = new ArrayDeque<>();
    private String current_tree_;
    private NodeSpec current_root_ = null;

    private TreeBuilder(String mainTreeId) {
        main_tree_id_ = mainTreeId;
        current_tree_ = mainTreeId;
    }

    /** Starts a document whose main tree is {@code mainTreeId}. */
    public static TreeBuilder create(String mainTreeId) {
        return new TreeBuilder(mainTreeId);
    }

    /** Finishes the current tree and starts another {@code <BehaviorTree>} (e.g. for SubTrees). */
    public TreeBuilder tree(String treeId) {
        finishTree();
        if (trees_.containsKey(treeId)) {
            throw new BtException("Tree '" + treeId + "' already defined");
        }
        current_tree_ = treeId;
        return this;
    }

    /** Opens a control/decorator node; close it with {@link #end()}. */
    public TreeBuilder begin(String id, String... ports) {
        push(id, ports, null);
        return this;
    }

    /** Adds a leaf node. */
    public TreeBuilder node(String id, String... ports) {
        push(id, ports, null);
        return end();
    }

    /** Adds a {@code <SubTree ID=treeId>} reference; ports are its remappings. */
    public TreeBuilder subTree(String treeId, String... ports) {
        push(NodeSpec.SUBTREE, ports, treeId);
        return end();
    }

    /** Closes the most recently opened node. */
    public TreeBuilder end() {
        if (stack_.isEmpty()) {
            throw new BtException("end() without an open node");
        }
        NodeSpec spec = stack_.pop().toSpec();
        if (stack_.isEmpty()) {
            if (current_root_ != null) {
                throw new BtException("Tree '" + current_tree_ + "' already has a root node");
            }
            current_root_ = spec;
        } else {
            stack_.peek().children.add(spec);
        }
        return this;
    }

    public TreeSpec build() {
        finishTree();
        return new TreeSpec(main_tree_id_, trees_, List.of(), List.of());
    }

    private void push(String id, String[] ports, String subtreeId) {
        if (ports.length % 2 != 0) {
            throw new BtException("Ports of '" + id + "' must be key/value pairs");
        }
        if (stack_.isEmpty() && current_root_ != null) {
            throw new BtException("Tree '" + current_tree_ + "' already has a root node");
        }
        String name = null;
        Map<String, String> attributes = new LinkedHashMap<>();
        for (int i = 0; i < ports.length; i += 2) {
            if ("name".equals(ports[i])) {
                name = ports[i + 1];
            } else {
                attributes.put(ports[i], ports[i + 1]);
            }
        }
        stack_.push(new Frame(id, name, attributes, subtreeId));
    }

    private void finishTree() {
        if (!stack_.isEmpty()) {
            throw new BtException(
                    "Tree '" + current_tree_ + "' has " + stack_.size() + " unclosed node(s)");
        }
        if (current_root_ == null) {
            throw new BtException("Tree '" + current_tree_ + "' has no root node");
        }
        trees_.put(current_tree_, current_root_);
        current_root_ = null;
    }
}
