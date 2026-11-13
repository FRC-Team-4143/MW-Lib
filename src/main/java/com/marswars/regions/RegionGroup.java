package com.marswars.regions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Objects;

/**
 * A named set of regions, such as all the places the robot should hold game pieces. A point is in
 * the group if it's in any of its regions. Each region still flips on its own, so a group is always
 * right for the current alliance.
 */
public final class RegionGroup {
    private final String name_;
    private final FieldRegion[] regions_;

    private RegionGroup(String name, FieldRegion[] regions) {
        name_ = name;
        regions_ = regions;
    }

    /** Makes a group from one or more regions. */
    public static RegionGroup of(String name, FieldRegion... regions) {
        Objects.requireNonNull(name, "name");
        if (regions.length == 0) {
            throw new IllegalArgumentException("Group " + name + " has no regions");
        }
        for (FieldRegion region : regions) {
            Objects.requireNonNull(region, "region");
        }
        return new RegionGroup(name, regions.clone());
    }

    /** Returns whether a robot pose is in any region in the group. */
    public boolean contains(Pose2d pose) {
        return contains(pose.getX(), pose.getY());
    }

    /** Returns whether a point is in any region in the group. */
    public boolean contains(Translation2d point) {
        return contains(point.getX(), point.getY());
    }

    /** Returns whether a point is in any region in the group. */
    public boolean contains(double x, double y) {
        for (FieldRegion region : regions_) {
            if (region.contains(x, y)) {
                return true;
            }
        }
        return false;
    }

    /** The group's name. */
    public String name() {
        return name_;
    }

    /** The regions in the group. */
    public List<FieldRegion> regions() {
        return List.of(regions_);
    }

    @Override
    public String toString() {
        return "RegionGroup[" + name_ + ", " + regions_.length + " regions]";
    }
}
