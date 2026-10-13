package com.marswars.bt.action;

import com.marswars.bt.core.NodeConfig;

/** Waits {@code msec} milliseconds, then succeeds. BT.CPP {@code Sleep}. */
public class SleepNode extends TimedWaitNode {
    public static final String MSEC = "msec";

    public SleepNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected double durationSeconds() {
        return getDouble(MSEC) / 1000.0;
    }
}
