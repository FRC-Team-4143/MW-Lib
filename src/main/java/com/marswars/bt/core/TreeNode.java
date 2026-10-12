package com.marswars.bt.core;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * Base class of every behavior-tree node. Mirrors BehaviorTree.CPP v4 {@code TreeNode}: a node is
 * ticked through {@link #executeTick()}, reports a {@link NodeStatus}, can be halted while RUNNING,
 * and reads its ports (literals or blackboard pointers) through the typed getters.
 *
 * <p>Status rules: {@link #tick()} may never return IDLE; a SKIPPED result is reported to the
 * parent but leaves the node IDLE; {@link #resetStatus()} is the only way back to IDLE.
 */
public abstract class TreeNode {

    /** Receives every status change of a node, with the tree clock time. */
    @FunctionalInterface
    public interface StatusListener {
        void onStatusChange(TreeNode node, NodeStatus previous, NodeStatus next, double timestamp);
    }

    private final String name_;
    private final NodeConfig config_;
    private final String registration_id_;
    private NodeStatus status_ = NodeStatus.IDLE;
    private int uid_ = 0;
    private StatusListener listener_ = null;
    // Port reads happen every tick: cache parsed literals and resolved pointer keys per port (the
    // config is immutable, so neither ever changes). Trees tick on one thread.
    private Map<String, Object> literal_cache_ = null;
    private Map<String, String> pointer_keys_ = null;

    protected TreeNode(String name, NodeConfig config) {
        config_ = config;
        registration_id_ = config.model() != null ? config.model().id() : defaultRegistrationId();
        name_ = (name == null || name.isEmpty()) ? registration_id_ : name;
    }

    /** Registration ID used when no model is attached: class name without the "Node" suffix. */
    protected String defaultRegistrationId() {
        String n = getClass().getSimpleName();
        if (n.endsWith("Node") && n.length() > 4) {
            n = n.substring(0, n.length() - 4);
        }
        return n;
    }

    // ------------------------------------------------------------------ ticking / status

    /** Ticks the node, validates the result, updates the status and returns it. */
    public final NodeStatus executeTick() {
        NodeStatus result = tick();
        if (result == null || result == NodeStatus.IDLE) {
            throw new BtException("Node [" + getPath() + "] returned IDLE from tick()");
        }
        validateTickResult(result);
        if (result == NodeStatus.SKIPPED) {
            if (status_ != NodeStatus.IDLE) {
                resetStatus();
            }
        } else {
            setStatus(result);
        }
        return result;
    }

    /** The node's own logic; called by {@link #executeTick()}. */
    protected abstract NodeStatus tick();

    /** Hook for subclasses that forbid some results (sync actions and conditions reject RUNNING). */
    protected void validateTickResult(NodeStatus result) {}

    /** Node category. */
    public abstract NodeKind kind();

    /**
     * Stops a RUNNING node (and its subtree) and resets its status to IDLE. Subclasses extend this
     * to halt children or call {@code onHalted()}; every override must end IDLE.
     */
    public void halt() {
        resetStatus();
    }

    /** {@link #halt()} plus a guarantee that the node ends IDLE. */
    public final void haltNode() {
        halt();
        if (status_ != NodeStatus.IDLE) {
            resetStatus();
        }
    }

    public final NodeStatus getStatus() {
        return status_;
    }

    /** Back to IDLE (the only legal way to get there). */
    public final void resetStatus() {
        NodeStatus previous = status_;
        status_ = NodeStatus.IDLE;
        if (previous != NodeStatus.IDLE) {
            notifyListener(previous, NodeStatus.IDLE);
        }
    }

    protected final void setStatus(NodeStatus status) {
        if (status == NodeStatus.IDLE) {
            throw new BtException(
                    "Node [" + getPath() + "]: use resetStatus() instead of setStatus(IDLE)");
        }
        if (status == NodeStatus.SKIPPED) {
            throw new BtException("Node [" + getPath() + "]: SKIPPED is not a stored status");
        }
        NodeStatus previous = status_;
        status_ = status;
        if (previous != status) {
            notifyListener(previous, status);
        }
    }

    private void notifyListener(NodeStatus previous, NodeStatus next) {
        if (listener_ != null) {
            listener_.onStatusChange(this, previous, next, now());
        }
    }

    public final void setStatusListener(StatusListener listener) {
        listener_ = listener;
    }

    // ------------------------------------------------------------------ identity

    /** Instance label ({@code name="..."} in XML); defaults to the registration ID. */
    public final String getName() {
        return name_;
    }

    /** Registration ID: the XML tag / {@code ID} attribute this node was created from. */
    public final String getRegistrationId() {
        return registration_id_;
    }

    /** 1-based preorder index inside its {@link BehaviorTree}; 0 until the tree is built. */
    public final int getUid() {
        return uid_;
    }

    final void setUid(int uid) {
        uid_ = uid;
    }

    /** Diagnostic path from the tree root, or the name when none was assigned. */
    public final String getPath() {
        return config_.path().isEmpty() ? name_ : config_.path();
    }

    public final NodeConfig getConfig() {
        return config_;
    }

    /** Model this node was instantiated from, when created by a factory. */
    public final Optional<NodeModel> getModel() {
        return Optional.ofNullable(config_.model());
    }

    public final Blackboard blackboard() {
        return config_.blackboard();
    }

    /** Tree clock in seconds. */
    protected final double now() {
        return config_.clock().getAsDouble();
    }

    /** Direct children, in tick order (empty for leaves). */
    public List<TreeNode> childNodes() {
        return List.of();
    }

    // ------------------------------------------------------------------ ports

    /** Raw attribute string of an input port, falling back to the model default. */
    public final Optional<String> getRawInput(String port) {
        return Optional.ofNullable(rawInput(port));
    }

    private String rawInput(String port) {
        String raw = config_.inputPorts().get(port);
        if (raw == null && config_.model() != null) {
            raw = config_.model().port(port).map(PortInfo::defaultValue).orElse(null);
        }
        return raw;
    }

    /**
     * Typed value of an input port: a {@code {key}} pointer is resolved through the blackboard, a
     * literal is parsed (once; later reads reuse the parsed value). Empty when the port is unset
     * and has no default, or the key is absent.
     */
    public final <T> Optional<T> getInput(String port, Class<T> type) {
        return Optional.ofNullable(input(port, type));
    }

    /** {@link #getInput} without the Optional: null when missing. Allocation-free on a cache hit. */
    private <T> T input(String port, Class<T> type) {
        String raw = rawInput(port);
        if (raw == null) {
            return null;
        }
        try {
            if (PortValues.isPointer(raw)) {
                Object value = blackboard().get(pointerKey(port, raw));
                return PortValues.convert(value, type, port);
            }
            return literal(port, raw, type);
        } catch (BtException e) {
            throw new BtException("[" + getPath() + "] " + e.getMessage(), e);
        }
    }

    private String pointerKey(String port, String raw) {
        if (pointer_keys_ == null) {
            pointer_keys_ = new HashMap<>();
        }
        String key = pointer_keys_.get(port);
        if (key == null) {
            key = PortValues.pointerKey(raw, port);
            pointer_keys_.put(port, key);
        }
        return key;
    }

    private <T> T literal(String port, String raw, Class<T> type) {
        if (literal_cache_ == null) {
            literal_cache_ = new HashMap<>();
        }
        Object cached = literal_cache_.get(port);
        if (cached != null && type.isInstance(cached)) {
            return type.cast(cached);
        }
        T parsed = PortValues.parseLiteral(raw, type, port);
        literal_cache_.put(port, parsed);
        return parsed;
    }

    /** Like {@link #getInput} but throws when the value is missing. */
    public final <T> T getInputOrThrow(String port, Class<T> type) {
        T value = input(port, type);
        if (value == null) {
            throw new BtException("Missing input port '" + port + "' on [" + getPath() + "]");
        }
        return value;
    }

    public final double getDouble(String port) {
        return getInputOrThrow(port, Double.class);
    }

    public final double getDouble(String port, double fallback) {
        Double v = input(port, Double.class);
        return v == null ? fallback : v;
    }

    public final int getInt(String port) {
        return getInputOrThrow(port, Integer.class);
    }

    public final int getInt(String port, int fallback) {
        Integer v = input(port, Integer.class);
        return v == null ? fallback : v;
    }

    public final boolean getBoolean(String port) {
        return getInputOrThrow(port, Boolean.class);
    }

    public final boolean getBoolean(String port, boolean fallback) {
        Boolean v = input(port, Boolean.class);
        return v == null ? fallback : v;
    }

    public final String getString(String port) {
        return getInputOrThrow(port, String.class);
    }

    public final String getString(String port, String fallback) {
        String v = input(port, String.class);
        return v == null ? fallback : v;
    }

    public final <E extends Enum<E>> E getEnum(String port, Class<E> type) {
        return getInputOrThrow(port, type);
    }

    public final <E extends Enum<E>> E getEnum(String port, Class<E> type, E fallback) {
        E v = input(port, type);
        return v == null ? fallback : v;
    }

    /**
     * Writes an output port. The port's attribute names the blackboard entry: {@code {key}}, a plain
     * {@code key}, or {@code {=}} (same name as the port), as in BT.CPP.
     */
    public final void setOutput(String port, Object value) {
        String raw = config_.outputPorts().get(port);
        if (raw == null) {
            raw = config_.inputPorts().get(port); // inout ports may be given either way
        }
        if (raw == null || raw.isEmpty()) {
            throw new BtException("Output port '" + port + "' of [" + getPath() + "] is not set");
        }
        String key = PortValues.isPointer(raw) ? PortValues.pointerKey(raw, port) : raw;
        if ("=".equals(key)) {
            key = port;
        }
        blackboard().set(key, value);
    }

    @Override
    public String toString() {
        return registration_id_ + "[" + name_ + "]#" + uid_ + ":" + status_;
    }
}
