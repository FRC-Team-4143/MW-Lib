package com.marswars.regions;

import java.util.Objects;

/**
 * The field a set of regions is drawn on: its size, its symmetry and the coordinate system the
 * robot code uses.
 *
 * <p>Shapes are always given in blue-corner coordinates, in meters: origin at the blue alliance
 * wall's right corner, +x toward red. {@code origin} is the system of the poses passed to {@code
 * contains} and of the outlines that get published. With {@link Origin#CENTER}, a queried point is
 * converted to blue-corner coordinates before it's tested, exactly as the Mariner editor does, so
 * the two always agree.
 *
 * @param lengthMeters the field's length along x
 * @param widthMeters the field's width along y
 * @param symmetry how the field maps from one alliance's side to the other's
 * @param origin the coordinate system of the robot code's poses
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

    /** Converts an x coordinate from the robot code's system to blue-corner coordinates. */
    public double toBlueCornerX(double x) {
        return origin == Origin.CENTER ? lengthMeters / 2 - x : x;
    }

    /** Converts a y coordinate from the robot code's system to blue-corner coordinates. */
    public double toBlueCornerY(double y) {
        return origin == Origin.CENTER ? widthMeters / 2 - y : y;
    }

    /**
     * Converts an x coordinate from blue-corner coordinates to the robot code's system. The
     * conversion is a half turn about the field center, so it's the same both ways.
     */
    public double fromBlueCornerX(double x) {
        return toBlueCornerX(x);
    }

    /** Converts a y coordinate from blue-corner coordinates to the robot code's system. */
    public double fromBlueCornerY(double y) {
        return toBlueCornerY(y);
    }

    /** Returns the matching x on the other alliance's side, in blue-corner coordinates. */
    public double flipX(double x) {
        return lengthMeters - x;
    }

    /** Returns the matching y on the other alliance's side, in blue-corner coordinates. */
    public double flipY(double y) {
        return symmetry == Symmetry.ROTATIONAL ? widthMeters - y : y;
    }
}
