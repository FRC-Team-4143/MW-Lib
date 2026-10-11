package com.marswars.bt.monitor;

import com.marswars.bt.core.BehaviorTree;
import com.marswars.logging.MwLog;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/**
 * Logs a tree's structure and per-tick status through {@link MwLog} (so it lands in the WPILOG, on
 * NT for AdvantageScope, and replays deterministically). Only what AdvantageScope can show goes
 * here; the tree itself (structure, XML, node list) is Arbor's to display, through the live view and
 * the {@code .btlog}. Keys, under {@code BehaviorTree/<name>/}:
 *
 * <ul>
 *   <li>{@code Status}: one status code per node in pre-order ({@code I R S F K}), on change
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
    }

    /** Default sink: {@link MwLog#log(String, String)}. */
    public static final Sink MWLOG = MwLog::log;

    /** Sink that publishes raw NT string topics {@code /<key>} (tests, tools). */
    public static final class NtSink implements Sink {
        private final NetworkTableInstance nt_;
        private final Map<String, StringPublisher> publishers_ = new HashMap<>();

        public NtSink(NetworkTableInstance nt) {
            nt_ = nt;
        }

        @Override
        public void putString(String key, String value) {
            publishers_
                    .computeIfAbsent(key, k -> nt_.getStringTopic("/" + k).publish())
                    .set(value);
        }
    }

    private final String name_;
    private final Sink sink_;
    private String last_status_ = null;

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

    /** The status of a freshly built tree; forces a publish even if it equals the last one. */
    public void publishStart(BehaviorTree tree) {
        last_status_ = null;
        publishStatus(tree);
    }

    /** Publishes the status string when it changed since the last call. */
    public void publishStatus(BehaviorTree tree) {
        String status = tree.statusString();
        if (!status.equals(last_status_)) {
            last_status_ = status;
            sink_.putString(key("Status"), status);
        }
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
