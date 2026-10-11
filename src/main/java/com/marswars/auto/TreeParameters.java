package com.marswars.auto;

import com.marswars.bt.core.Blackboard;
import com.marswars.bt.core.ParameterStore;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.TreeSpec;
import com.marswars.bt.monitor.NetworkParameterStore;
import java.util.Objects;

/**
 * One auto's tunable values: the ports declared for its main tree in {@code <TreeNodesModel>}.
 * Each lives in a {@link ParameterStore} under {@code Autos/<auto>/<port>} (dashboard {@code
 * /Tuning/Autos/...} by default), is published when the auto loads so it can be tuned while
 * disabled, and is copied into the root blackboard at the start of every run.
 */
final class TreeParameters {
    /** Shared dashboard-backed store; created lazily so unit tests never touch NetworkTables. */
    private static ParameterStore default_store_ = null;

    static synchronized ParameterStore defaultStore() {
        if (default_store_ == null) {
            default_store_ = new NetworkParameterStore();
        }
        return default_store_;
    }

    private final String auto_name_;
    private final ParameterStore store_;

    TreeParameters(String auto_name, ParameterStore store) {
        auto_name_ = Objects.requireNonNull(auto_name, "auto_name");
        store_ = Objects.requireNonNull(store, "store");
    }

    /** Store key of one parameter: {@code Autos/<auto>/<port>}. */
    String key(String port) {
        return "Autos/" + auto_name_ + "/" + port;
    }

    /** Creates a store entry (at its default) for each parameter the tree declares. */
    void publish(TreeSpec spec) {
        for (PortInfo port : spec.mainTreeParameters()) {
            if (port.defaultValue() != null) {
                store_.value(key(port.name()), port);
            }
        }
    }

    /** Writes each parameter's current value into a run's root blackboard. */
    void seed(TreeSpec spec, Blackboard root) {
        for (PortInfo port : spec.mainTreeParameters()) {
            if (port.defaultValue() != null) {
                root.set(port.name(), store_.value(key(port.name()), port));
            }
        }
    }
}
