package com.marswars.bt.monitor;

import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.TreeNode;
import com.marswars.logging.MwLog;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Logs a tree's structure and per-tick status through {@link MwLog} (so it lands in the WPILOG, on
 * NT for AdvantageScope, and replays deterministically). Only what AdvantageScope can show goes
 * here; the tree itself (structure, XML, node list) is Arbor's to display, through the live view and
 * the {@code .btlog}. Keys, under {@code BehaviorTree/<name>/}:
 *
 * <ul>
 *   <li>{@code Transitions}: every node status change since the last tick, one {@code "<node>:
 *       <STATE>"} entry per change in the order they happened (for example {@code
 *       "Follow::2: RUNNING"}), so AdvantageScope can show it as a table or timeline like a {@code
 *       .btlog}. Empty on ticks where nothing changed.
 *   <li>{@code Error}: last load/tick error, or empty
 *   <li>{@code Result}: final result when the tree stops ({@code SUCCESS}, {@code FAILURE}, {@code
 *       INTERRUPTED})
 * </ul>
 *
 * Plus the global {@code BehaviorTree/Active} (name of the running tree, or empty).
 */
public final class BehaviorTreeMonitor {
    public static final String PREFIX = "BehaviorTree/";

    /** Where string values go. */
    @FunctionalInterface
    public interface Sink {
        void putString(String key, String value);

        /** Lists of strings; sinks without arrays get one string, one entry per line. */
        default void putStringArray(String key, String[] values) {
            putString(key, String.join("\n", values));
        }
    }

    /** Default sink: {@link MwLog}. */
    public static final Sink MWLOG =
            new Sink() {
                @Override
                public void putString(String key, String value) {
                    MwLog.log(key, value);
                }

                @Override
                public void putStringArray(String key, String[] values) {
                    MwLog.log(key, values);
                }
            };

    /** Sink that publishes raw NT string topics {@code /<key>} (tests, tools). */
    public static final class NtSink implements Sink {
        private final NetworkTableInstance nt_;
        private final Map<String, StringPublisher> publishers_ = new HashMap<>();
        private final Map<String, StringArrayPublisher> array_publishers_ = new HashMap<>();

        public NtSink(NetworkTableInstance nt) {
            nt_ = nt;
        }

        @Override
        public void putString(String key, String value) {
            publishers_
                    .computeIfAbsent(key, k -> nt_.getStringTopic("/" + k).publish())
                    .set(value);
        }

        @Override
        public void putStringArray(String key, String[] values) {
            array_publishers_
                    .computeIfAbsent(key, k -> nt_.getStringArrayTopic("/" + k).publish())
                    .set(values);
        }
    }

    private final String name_;
    private final Sink sink_;
    private final List<String> pending_ = new ArrayList<>();
    private BehaviorTree tree_ = null;
    private TreeNode.StatusListener listener_ = null;
    private boolean published_empty_ = false;

    public BehaviorTreeMonitor(String treeName) {
        this(treeName, MWLOG);
    }

    public BehaviorTreeMonitor(String treeName, Sink sink) {
        name_ = Objects.requireNonNull(treeName, "treeName");
        sink_ = Objects.requireNonNull(sink, "sink");
    }

    public String name() {
        return name_;
    }

    public String key(String leaf) {
        return PREFIX + name_ + "/" + leaf;
    }

    /** Starts recording {@code tree}'s node transitions (and clears any left from a previous tree). */
    public void publishStart(BehaviorTree tree) {
        detach();
        tree_ = tree;
        pending_.clear();
        listener_ = (node, previous, next, timestamp) -> pending_.add(node.getPath() + ": " + next.name());
        tree.addStatusListener(listener_);
        published_empty_ = false; // make the next publish write even an empty list
    }

    /**
     * Publishes the transitions since the last call. On a tick with none it writes an empty list once,
     * so the same transition happening on two ticks in a row is still two entries in the log.
     */
    public void publishTransitions() {
        if (pending_.isEmpty()) {
            if (!published_empty_) {
                sink_.putStringArray(key("Transitions"), new String[0]);
                published_empty_ = true;
            }
            return;
        }
        sink_.putStringArray(key("Transitions"), pending_.toArray(new String[0]));
        pending_.clear();
        published_empty_ = false;
    }

    /** Stops listening to the tree (after its last {@link #publishTransitions()}). */
    public void detach() {
        if (tree_ != null && listener_ != null) {
            tree_.removeStatusListener(listener_);
        }
        tree_ = null;
        listener_ = null;
    }

    public void publishError(String message) {
        sink_.putString(key("Error"), message == null ? "" : message);
    }

    public void publishResult(String result) {
        sink_.putString(key("Result"), result);
    }

    /** Sets {@code BehaviorTree/Active} through this monitor's sink. */
    public void publishActive(String treeName) {
        publishActive(sink_, treeName);
    }

    public static void publishActive(Sink sink, String treeName) {
        sink.putString(PREFIX + "Active", treeName == null ? "" : treeName);
    }
}
