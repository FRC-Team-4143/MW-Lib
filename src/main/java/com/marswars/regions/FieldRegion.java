package com.marswars.regions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * A named area of the field, made of one or more shapes with optional holes cut out of it.
 *
 * <p>A point is in the region if it's in at least one added shape and in no subtracted shape. A
 * point exactly on a hole's edge is therefore outside the region.
 *
 * <p>Shapes are in blue-corner coordinates (see {@link FieldGeometry}). A {@link AllianceMode#FLIPS
 * FLIPS} region is drawn from blue's side. When the robot is on red, the queried point is flipped
 * to blue's side before it's tested, using the field's {@link Symmetry}. The alliance is read on every call (see {@link AllianceState}), so nothing needs to
 * be flipped or rebuilt when the alliance changes. An unknown alliance counts as blue.
 *
 * <p>Regions are immutable, and {@code contains} doesn't allocate, so it's safe to call every loop.
 *
 * <pre>{@code
 * FieldRegion ALLIANCE_ZONE =
 *         FieldRegion.builder("ALLIANCE_ZONE", FIELD)
 *                 .flips()
 *                 .add(Shape.box(0, 0, 3.963, 8.042))
 *                 .build();
 *
 * if (ALLIANCE_ZONE.contains(pose)) { ... }
 * }</pre>
 */
public final class FieldRegion {
    private final String name_;
    private final FieldGeometry field_;
    private final AllianceMode alliance_mode_;
    private final Shape[] added_;
    private final Shape[] subtracted_;

    private FieldRegion(Builder builder) {
        name_ = builder.name_;
        field_ = builder.field_;
        alliance_mode_ = builder.alliance_mode_;
        added_ = builder.added_.toArray(new Shape[0]);
        subtracted_ = builder.subtracted_.toArray(new Shape[0]);
    }

    /**
     * Starts building a region. Regions flip by default; call {@link Builder#fixed()} for one that
     * stays put.
     *
     * @param name the region's name, used for dashboard topics
     * @param field the field the region is drawn on
     */
    public static Builder builder(String name, FieldGeometry field) {
        return new Builder(name, field);
    }

    /** Returns whether a robot pose is in the region, for the robot's current alliance. */
    public boolean contains(Pose2d pose) {
        return contains(pose.getX(), pose.getY());
    }

    /** Returns whether a point is in the region, for the robot's current alliance. */
    public boolean contains(Translation2d point) {
        return contains(point.getX(), point.getY());
    }

    /**
     * Returns whether a point is in the region, for the robot's current alliance. The point is in
     * the robot code's coordinate system ({@link FieldGeometry#origin()}).
     */
    public boolean contains(double x, double y) {
        double bx = field_.toBlueCornerX(x);
        double by = field_.toBlueCornerY(y);
        if (alliance_mode_ == AllianceMode.FLIPS && AllianceState.isRed()) {
            return containsAsDrawn(field_.flipX(bx), field_.flipY(by));
        }
        return containsAsDrawn(bx, by);
    }

    /**
     * Returns whether a point in blue-corner coordinates is in the region as drawn, ignoring the
     * alliance.
     */
    public boolean containsAsDrawn(double x, double y) {
        for (Shape hole : subtracted_) {
            if (hole.contains(x, y)) {
                return false;
            }
        }
        for (Shape shape : added_) {
            if (shape.contains(x, y)) {
                return true;
            }
        }
        return false;
    }

    /** The region's name. */
    public String name() {
        return name_;
    }

    /** The field the region is drawn on. */
    public FieldGeometry field() {
        return field_;
    }

    /** Whether the region flips with the alliance. */
    public AllianceMode allianceMode() {
        return alliance_mode_;
    }

    /**
     * Returns the outline of every shape and hole, each as a closed loop, where they are for the
     * robot's current alliance, in the robot code's coordinate system. This allocates, so call it when the alliance changes, not every
     * loop. {@link RegionPublisher} does this for you.
     */
    public List<Translation2d[]> outlines() {
        boolean flip = alliance_mode_ == AllianceMode.FLIPS && AllianceState.isRed();
        List<Translation2d[]> outlines = new ArrayList<>(added_.length + subtracted_.length);
        for (Shape shape : added_) {
            outlines.add(toRobot(shape.outline(), flip));
        }
        for (Shape shape : subtracted_) {
            outlines.add(toRobot(shape.outline(), flip));
        }
        return Collections.unmodifiableList(outlines);
    }

    /** Moves outline points to red's side if needed, then into the robot code's coordinates. */
    private Translation2d[] toRobot(Translation2d[] points, boolean flip) {
        if (!flip && field_.origin() == Origin.BLUE_CORNER) {
            return points;
        }
        Translation2d[] moved = new Translation2d[points.length];
        for (int i = 0; i < points.length; i++) {
            double x = points[i].getX();
            double y = points[i].getY();
            if (flip) {
                x = field_.flipX(x);
                y = field_.flipY(y);
            }
            moved[i] = new Translation2d(field_.fromBlueCornerX(x), field_.fromBlueCornerY(y));
        }
        return moved;
    }

    @Override
    public String toString() {
        return "FieldRegion[" + name_ + ", " + alliance_mode_ + "]";
    }

    /** Builds a {@link FieldRegion}. */
    public static final class Builder {
        private final String name_;
        private final FieldGeometry field_;
        private AllianceMode alliance_mode_ = AllianceMode.FLIPS;
        private final List<Shape> added_ = new ArrayList<>();
        private final List<Shape> subtracted_ = new ArrayList<>();

        private Builder(String name, FieldGeometry field) {
            name_ = Objects.requireNonNull(name, "name");
            field_ = Objects.requireNonNull(field, "field");
            if (name.isBlank()) {
                throw new IllegalArgumentException("A region needs a name");
            }
        }

        /** The region is drawn from blue's side and moves to red's side on red. The default. */
        public Builder flips() {
            alliance_mode_ = AllianceMode.FLIPS;
            return this;
        }

        /** The region stays in the same place on the field for both alliances. */
        public Builder fixed() {
            alliance_mode_ = AllianceMode.FIXED;
            return this;
        }

        /** Adds shapes to the region. */
        public Builder add(Shape... shapes) {
            for (Shape shape : shapes) {
                added_.add(Objects.requireNonNull(shape, "shape"));
            }
            return this;
        }

        /** Cuts holes out of the region. */
        public Builder subtract(Shape... shapes) {
            for (Shape shape : shapes) {
                subtracted_.add(Objects.requireNonNull(shape, "shape"));
            }
            return this;
        }

        /** Builds the region. It needs at least one added shape. */
        public FieldRegion build() {
            if (added_.isEmpty()) {
                throw new IllegalStateException("Region " + name_ + " has no shapes");
            }
            return new FieldRegion(this);
        }
    }
}
