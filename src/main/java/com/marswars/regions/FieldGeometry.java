package com.marswars.regions;

import java.util.Objects;

/**
 * The field a set of regions is drawn on: its size, its symmetry and the coordinate system the
 * robot code uses. Shapes and queried points are both in that coordinate system, in meters.
 *
 * @param lengthMeters the field's length along x
 * @param widthMeters the field's width along y
 * @param symmetry how the field maps from one alliance's side to the other's
 * @param origin where (0, 0) is
 */
public record FieldGeometry(
        double lengthMeters, double widthMeters, Symmetry symmetry, Origin origin) {

    public FieldGeometry {
        if (!(lengthMeters > 0) || !(widthMeters > 0)) {
            throw new IllegalArgumentException("Field size must be positive");
        }
        Objects.requireNonNull(symmetry, "symmetry");
        Objects.requireNonNull(origin, "origin");
    }

    /** A field with the origin at the blue corner, as WPILib uses by default. */
    public FieldGeometry(double lengthMeters, double widthMeters, Symmetry symmetry) {
        this(lengthMeters, widthMeters, symmetry, Origin.BLUE_CORNER);
    }

    /** Returns the x coordinate of the matching point on the other alliance's side. */
    public double flipX(double x) {
        return origin == Origin.BLUE_CORNER ? lengthMeters - x : -x;
    }

    /** Returns the y coordinate of the matching point on the other alliance's side. */
    public double flipY(double y) {
        if (symmetry == Symmetry.MIRRORED) {
            return y;
        }
        return origin == Origin.BLUE_CORNER ? widthMeters - y : -y;
    }
}
