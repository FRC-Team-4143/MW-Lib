package com.marswars.bt.support;

import java.util.function.DoubleSupplier;

/** Manually advanced clock in seconds. */
public final class FakeClock implements DoubleSupplier {
    private double now_ = 0.0;

    @Override
    public double getAsDouble() {
        return now_;
    }

    public void advance(double seconds) {
        now_ += seconds;
    }

    public void set(double seconds) {
        now_ = seconds;
    }
}
