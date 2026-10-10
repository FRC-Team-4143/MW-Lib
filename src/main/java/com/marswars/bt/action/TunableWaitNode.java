package com.marswars.bt.action;

import com.marswars.bt.core.NodeConfig;
import java.util.Objects;

/**
 * Waits a live-tunable number of seconds, published under {@code /Tuning/<key>} with {@code
 * default_seconds} as the initial value. The value is read when the node starts. Replaces the
 * SmartDashboard-based {@code DynamicWaitCommand}. MW-Lib extra.
 */
public class TunableWaitNode extends TimedWaitNode {
    public static final String KEY = "key";
    public static final String DEFAULT_SECONDS = "default_seconds";

    private final TunableRegistry registry_;

    public TunableWaitNode(String name, NodeConfig config, TunableRegistry registry) {
        super(name, config);
        registry_ = Objects.requireNonNull(registry, "registry");
    }

    @Override
    protected double durationSeconds() {
        return registry_.register(getString(KEY), getDouble(DEFAULT_SECONDS)).getAsDouble();
    }
}
