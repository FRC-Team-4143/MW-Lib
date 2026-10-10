package com.marswars.bt.core;

import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;

/**
 * Key/value store shared by the nodes of a tree, with BehaviorTree.CPP v4 scoping rules: every
 * SubTree gets a child blackboard that can remap individual entries to its parent, optionally
 * auto-remap every key ({@code _autoremap="true"}), and any key prefixed with {@code '@'} addresses
 * the root blackboard directly.
 *
 * <p>Thread-safe: trees tick on the robot thread while debug servers read entries from theirs.
 * Locks are only ever taken child-then-parent, so nested lookups cannot deadlock.
 */
public final class Blackboard {
    /** Prefix that routes a key to the root blackboard ({@code {@trajectories}}). */
    public static final char ROOT_PREFIX = '@';

    private final Blackboard parent_;
    private final Map<String, Object> storage_ = new LinkedHashMap<>();
    private final Map<String, String> remapping_ = new HashMap<>();
    private boolean autoremap_ = false;

    private Blackboard(Blackboard parent) {
        parent_ = parent;
    }

    public static Blackboard createRoot() {
        return new Blackboard(null);
    }

    public static Blackboard createChild(Blackboard parent) {
        return new Blackboard(Objects.requireNonNull(parent, "parent"));
    }

    public Blackboard parent() {
        return parent_;
    }

    public Blackboard root() {
        Blackboard b = this;
        while (b.parent_ != null) {
            b = b.parent_;
        }
        return b;
    }

    /** When enabled, keys missing locally fall through to the parent for both get and set. */
    public synchronized void enableAutoRemapping(boolean enable) {
        autoremap_ = enable;
    }

    public synchronized boolean isAutoRemapping() {
        return autoremap_;
    }

    /** Routes {@code internal} (this blackboard) to {@code external} (the parent blackboard). */
    public synchronized void addSubtreeRemapping(String internal, String external) {
        remapping_.put(internal, external);
    }

    public synchronized Map<String, String> remappings() {
        return Map.copyOf(remapping_);
    }

    /** Resolved value, or {@code null} when absent. */
    public synchronized Object get(String key) {
        if (isRootKey(key)) {
            return root().get(key.substring(1));
        }
        String external = remapping_.get(key);
        if (external != null && parent_ != null) {
            return parent_.get(external);
        }
        if (storage_.containsKey(key)) {
            return storage_.get(key);
        }
        if (autoremap_ && parent_ != null) {
            return parent_.get(key);
        }
        return null;
    }

    /** Resolved value converted to {@code type} (strings are parsed), or {@code null}. */
    public <T> T get(String key, Class<T> type) {
        Object value = get(key);
        return value == null ? null : PortValues.convert(value, type, "blackboard entry '" + key + "'");
    }

    public Optional<Object> getOptional(String key) {
        return Optional.ofNullable(get(key));
    }

    public synchronized boolean contains(String key) {
        if (isRootKey(key)) {
            return root().contains(key.substring(1));
        }
        String external = remapping_.get(key);
        if (external != null && parent_ != null) {
            return parent_.contains(external);
        }
        if (storage_.containsKey(key)) {
            return true;
        }
        return autoremap_ && parent_ != null && parent_.contains(key);
    }

    /** Writes through remappings/autoremap when the key lives in a parent; otherwise stores locally. */
    public synchronized void set(String key, Object value) {
        if (isRootKey(key)) {
            root().set(key.substring(1), value);
            return;
        }
        String external = remapping_.get(key);
        if (external != null && parent_ != null) {
            parent_.set(external, value);
            return;
        }
        if (!storage_.containsKey(key) && autoremap_ && parent_ != null && parent_.contains(key)) {
            parent_.set(key, value);
            return;
        }
        storage_.put(key, value);
    }

    /** Stores locally, shadowing any parent entry (used for literal SubTree port values). */
    public synchronized void setLocal(String key, Object value) {
        storage_.put(key, value);
    }

    /** Snapshot of the keys stored in this blackboard (not its parents). */
    public synchronized Set<String> localKeys() {
        return Collections.unmodifiableSet(new java.util.LinkedHashSet<>(storage_.keySet()));
    }

    /** Snapshot of the entries stored in this blackboard (not its parents); values may be null. */
    public synchronized Map<String, Object> localEntries() {
        return Collections.unmodifiableMap(new LinkedHashMap<>(storage_));
    }

    public synchronized void clear() {
        storage_.clear();
    }

    private static boolean isRootKey(String key) {
        return key != null && !key.isEmpty() && key.charAt(0) == ROOT_PREFIX;
    }
}
