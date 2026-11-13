package com.marswars.regions;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/**
 * The robot's alliance, as regions see it. It reads {@link DriverStation#getAlliance()} at most once
 * every {@value #REFRESH_MILLIS} ms and caches the answer, so checking it every loop is cheap. An
 * unknown alliance (no driver station yet) counts as blue.
 */
public final class AllianceState {
    /** How long a reading is reused before the driver station is asked again. */
    public static final long REFRESH_MILLIS = 20;

    private static final long REFRESH_NANOS = REFRESH_MILLIS * 1_000_000L;

    private static volatile boolean red_ = false;
    private static volatile boolean read_once_ = false;
    private static volatile long last_read_nanos_ = 0;
    private static volatile long change_count_ = 0;
    private static volatile boolean force_refresh_ = false;
    private static volatile boolean overridden_ = false;
    private static volatile Optional<Alliance> override_ = Optional.empty();

    private AllianceState() {}

    /** Returns whether the robot is on red. Unknown counts as blue. */
    public static boolean isRed() {
        refreshIfStale();
        return red_;
    }

    /**
     * Returns a number that goes up each time the alliance changes. {@link RegionPublisher} uses it
     * to republish outlines only on a change.
     */
    public static long changeCount() {
        refreshIfStale();
        return change_count_;
    }

    /** Reads the alliance now, whether or not the cached reading is stale. */
    public static synchronized void refresh() {
        Optional<Alliance> alliance = overridden_ ? override_ : DriverStation.getAlliance();
        boolean red = alliance.isPresent() && alliance.get() == Alliance.Red;
        if (read_once_ && red != red_) {
            change_count_++;
        }
        red_ = red;
        read_once_ = true;
        last_read_nanos_ = System.nanoTime();
        force_refresh_ = false;
    }

    private static void refreshIfStale() {
        if (!read_once_ || force_refresh_ || System.nanoTime() - last_read_nanos_ >= REFRESH_NANOS) {
            refresh();
        }
    }

    /**
     * Uses this alliance instead of the driver station's, for tests and simulation. An empty
     * alliance means unknown.
     */
    public static void overrideForTesting(Optional<Alliance> alliance) {
        override_ = alliance;
        overridden_ = true;
        refresh();
    }

    /** Goes back to reading the driver station. */
    public static void clearOverride() {
        overridden_ = false;
        override_ = Optional.empty();
        force_refresh_ = true;
    }
}
