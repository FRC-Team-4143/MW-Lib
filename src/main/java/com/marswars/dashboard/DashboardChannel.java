package com.marswars.dashboard;

/**
 * Describes one NetworkTables channel exposed by a {@link DashboardBridge}: its wire name,
 * primitive type, and whether the dashboard is allowed to write to it.
 *
 * <p>{@code BIDIRECTIONAL} channels subscribe on the bridge's ToRobot table (the dashboard writes,
 * the robot reads) and mirror the robot's authoritative value back out on the ToDashboard table.
 * {@code OUTPUT_ONLY} channels only publish on ToDashboard -- there is nothing for the dashboard to
 * tell the robot (e.g. whether the current match is an elimination match, which the robot
 * determines for itself).
 */
public record DashboardChannel(String name, Type type, Direction direction) {

    public enum Type {
        INTEGER,
        BOOLEAN
    }

    public enum Direction {
        BIDIRECTIONAL,
        OUTPUT_ONLY
    }

    // TODO(human): implement these four static factory methods.
    //
    // Each one is a single call to the DashboardChannel constructor above, fixing two of the three
    // fields. For example:
    //
    //     public static DashboardChannel bidirectionalInt(String name) {
    //         return new DashboardChannel(name, Type.INTEGER, Direction.BIDIRECTIONAL);
    //     }
    //
    // Why these exist: without them, every channel declaration in a per-year robot repo would have
    // to spell out
    //     new DashboardChannel("l2_state", DashboardChannel.Type.INTEGER, DashboardChannel.Direction.BIDIRECTIONAL)
    // every time. These factories are what let consumer code write the much shorter
    //     DashboardChannel.bidirectionalInt("l2_state")
    // instead.

    public static DashboardChannel bidirectionalInt(String name) {
        return new DashboardChannel(name, Type.INTEGER, Direction.BIDIRECTIONAL);
    }

    public static DashboardChannel outputInt(String name) {
        return new DashboardChannel(name, Type.INTEGER, Direction.OUTPUT_ONLY);
    }

    public static DashboardChannel bidirectionalBool(String name) {
        return new DashboardChannel(name, Type.BOOLEAN, Direction.BIDIRECTIONAL);
    }

    public static DashboardChannel outputBool(String name) {
        return new DashboardChannel(name, Type.BOOLEAN, Direction.OUTPUT_ONLY);
    }
}
