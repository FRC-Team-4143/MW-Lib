package com.marswars.bt.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * A built tree: the root node, the root blackboard, the tree clock and the flat preorder node list
 * (uid = index + 1) that monitors use to describe structure and status compactly.
 *
 * <p>Tick it once per robot loop with {@link #tickOnce()}; the caller decides what to do when the
 * root completes. {@link #haltTree()} stops every RUNNING node (for example when the auto command is
 * interrupted).
 */
public final class BehaviorTree {
    private final String main_tree_id_;
    private final TreeNode root_;
    private final Blackboard blackboard_;
    private final DoubleSupplier clock_;
    private final String xml_;
    private final List<TreeNode> nodes_ = new ArrayList<>();
    private final List<Subtree> subtrees_ = new ArrayList<>();
    private final java.util.Map<TreeNode, TreeNode> parents_ = new java.util.IdentityHashMap<>();
    private final List<TreeNode.StatusListener> listeners_ =
            new java.util.concurrent.CopyOnWriteArrayList<>();
    private long tick_count_ = 0;

    /**
     * One instantiated (sub)tree and its blackboard, like BT.CPP {@code Tree::Subtree}.
     *
     * @param instanceName full path of the SubTree node ("" for the main tree)
     * @param treeId the {@code <BehaviorTree ID>} it was instantiated from
     * @param blackboard the blackboard its nodes use
     */
    public record Subtree(String instanceName, String treeId, Blackboard blackboard) {}

    public BehaviorTree(
            String mainTreeId, TreeNode root, Blackboard blackboard, DoubleSupplier clock, String xml) {
        this(mainTreeId, root, blackboard, clock, xml, List.of());
    }

    /**
     * @param subtrees every instantiated SubTree (main tree excluded; it is added first
     *     automatically)
     */
    public BehaviorTree(
            String mainTreeId,
            TreeNode root,
            Blackboard blackboard,
            DoubleSupplier clock,
            String xml,
            List<Subtree> subtrees) {
        main_tree_id_ = mainTreeId == null ? "" : mainTreeId;
        root_ = Objects.requireNonNull(root, "root");
        blackboard_ = Objects.requireNonNull(blackboard, "blackboard");
        clock_ = Objects.requireNonNull(clock, "clock");
        xml_ = xml;
        subtrees_.add(new Subtree("", main_tree_id_, blackboard_));
        subtrees_.addAll(subtrees);
        collect(root_, null);
        for (int i = 0; i < nodes_.size(); i++) {
            nodes_.get(i).setUid(i + 1);
            nodes_.get(i).setStatusListener(this::dispatch);
        }
    }

    public BehaviorTree(TreeNode root, Blackboard blackboard, DoubleSupplier clock) {
        this("", root, blackboard, clock, null);
    }

    private void collect(TreeNode node, TreeNode parent) {
        nodes_.add(node);
        parents_.put(node, parent);
        for (TreeNode child : node.childNodes()) {
            collect(child, node);
        }
    }

    private void dispatch(TreeNode node, NodeStatus prev, NodeStatus next, double t) {
        for (TreeNode.StatusListener l : listeners_) {
            l.onStatusChange(node, prev, next, t);
        }
    }

    /**
     * Ticks the root once. A root that completed on the previous tick is reset first, so the final
     * SUCCESS/FAILURE stays visible to monitors for one loop and the tree restarts cleanly.
     */
    public NodeStatus tickOnce() {
        if (root_.getStatus().isCompleted()) {
            root_.resetStatus();
        }
        tick_count_++;
        return root_.executeTick();
    }

    /** Halts the root, then every node as a safety net, leaving the whole tree IDLE. */
    public void haltTree() {
        root_.haltNode();
        for (TreeNode node : nodes_) {
            if (node.getStatus() != NodeStatus.IDLE) {
                node.haltNode();
            }
        }
    }

    /** Alias of {@link #haltTree()}; node-internal counters (Repeat, RunOnce) are not cleared. */
    public void reset() {
        haltTree();
    }

    public TreeNode getRoot() {
        return root_;
    }

    public Blackboard getBlackboard() {
        return blackboard_;
    }

    public DoubleSupplier getClock() {
        return clock_;
    }

    public String getMainTreeId() {
        return main_tree_id_;
    }

    /** The XML this tree was built from, when it came from a file or text. */
    public Optional<String> getXml() {
        return Optional.ofNullable(xml_);
    }

    /** Every node in preorder; {@code getNodes().get(i).getUid() == i + 1}. */
    public List<TreeNode> getNodes() {
        return Collections.unmodifiableList(nodes_);
    }

    /** Main tree first, then every SubTree instance in creation order. */
    public List<Subtree> getSubtrees() {
        return Collections.unmodifiableList(subtrees_);
    }

    public long getTickCount() {
        return tick_count_;
    }

    /** One status code per node in preorder, e.g. {@code "RSIR"} (see {@link NodeStatus#code()}). */
    public String statusString() {
        StringBuilder sb = new StringBuilder(nodes_.size());
        for (TreeNode node : nodes_) {
            sb.append(node.getStatus().code());
        }
        return sb.toString();
    }

    /** Parent of {@code node} in this tree, or empty for the root. */
    public Optional<TreeNode> getParent(TreeNode node) {
        return Optional.ofNullable(parents_.get(node));
    }

    /** Adds a listener for every status transition of every node (thread-safe list). */
    public void addStatusListener(TreeNode.StatusListener listener) {
        listeners_.add(Objects.requireNonNull(listener, "listener"));
    }

    public void removeStatusListener(TreeNode.StatusListener listener) {
        listeners_.remove(listener);
    }

    /** Replaces every listener with {@code listener} ({@code null} clears them). */
    public void setStatusListener(TreeNode.StatusListener listener) {
        listeners_.clear();
        if (listener != null) {
            listeners_.add(listener);
        }
    }
}
