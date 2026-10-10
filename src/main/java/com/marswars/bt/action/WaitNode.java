package com.marswars.bt.action;

import com.marswars.bt.core.NodeConfig;

/** Waits {@code seconds}, then succeeds. MW-Lib extra (BT.CPP's equivalent is {@code Sleep}). */
public class WaitNode extends TimedWaitNode {
    public static final String SECONDS = "seconds";

    public WaitNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected double durationSeconds() {
        return getDouble(SECONDS);
    }
}
