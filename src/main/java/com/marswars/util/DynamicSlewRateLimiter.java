package com.marswars.util;

import org.wpilib.math.util.MathUtil;
import org.wpilib.system.Timer;

/**
 * A slew rate limiter that allows the rate limit to be changed dynamically without losing state.
 * This is based on WPILib's SlewRateLimiter but exposes methods to update the rate limit.
 */
public class DynamicSlewRateLimiter {
    private double rate_limit_;
    private double prev_val_;
    private double prev_time_;

    /**
     * Creates a new DynamicSlewRateLimiter with the given rate limit.
     *
     * @param rate_limit The rate-of-change limit, in units per second.
     */
    public DynamicSlewRateLimiter(double rate_limit) {
        this.rate_limit_ = rate_limit;
        this.prev_val_ = 0.0;
        this.prev_time_ = Timer.getTimestamp();
    }

    /**
     * Creates a new DynamicSlewRateLimiter with the given rate limit and initial value.
     *
     * @param rate_limit The rate-of-change limit, in units per second.
     * @param initial_value The initial value of the input.
     */
    public DynamicSlewRateLimiter(double rate_limit, double initial_value) {
        this.rate_limit_ = rate_limit;
        this.prev_val_ = initial_value;
        this.prev_time_ = Timer.getTimestamp();
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input) {
        double current_time = Timer.getTimestamp();
        double elapsed_time = current_time - prev_time_;
        prev_time_ = current_time;
        prev_val_ =
                prev_val_
                        + Math.clamp(
                                input - prev_val_,
                                -rate_limit_ * elapsed_time,
                                rate_limit_ * elapsed_time);
        return prev_val_;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        prev_val_ = value;
        prev_time_ = Timer.getTimestamp();
    }

    /**
     * Sets the rate limit.
     *
     * @param rate_limit The new rate-of-change limit, in units per second.
     */
    public void setRateLimit(double rate_limit) {
        this.rate_limit_ = rate_limit;
    }

    /**
     * Gets the current rate limit.
     *
     * @return The current rate-of-change limit, in units per second.
     */
    public double getRateLimit() {
        return rate_limit_;
    }
}
