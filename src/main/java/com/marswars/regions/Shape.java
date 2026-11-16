package com.marswars.regions;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;

/**
 * A shape on the field: a rotated rectangle, a circle or a polygon. Shapes are immutable, and
 * {@link #contains(double, double)} doesn't allocate, so it's safe to call every loop.
 *
 * <p>A point on a shape's edge counts as inside it, within {@link #EDGE_TOLERANCE_METERS}. These
 * rules match the Mariner editor exactly, so what you draw is what the robot checks.
 */
public sealed interface Shape permits Shape.Rect, Shape.Circle, Shape.Polygon {

    /** How far outside an edge a point can be and still count as on it. */
    double EDGE_TOLERANCE_METERS = 1e-9;

    /** Points used to draw a circle's outline. */
    int CIRCLE_OUTLINE_POINTS = 64;

    /** Returns whether a point is inside this shape or on its edge. */
    boolean contains(double x, double y);

    /** Returns this shape's outline as a closed loop (the first point repeated at the end). */
    Translation2d[] outline();

    /**
     * A rectangle centered on (cx, cy), rotated counterclockwise about its center.
     *
     * @param cx center x, meters
     * @param cy center y, meters
     * @param width size along x before rotating, meters
     * @param height size along y before rotating, meters
     * @param rotationDeg counterclockwise rotation, degrees
     */
    static Shape rect(double cx, double cy, double width, double height, double rotationDeg) {
        double[] cosSin = cosSin(rotationDeg);
        return new Rect(cx, cy, width, height, cosSin[0], cosSin[1]);
    }

    /**
     * A rotated rectangle given the cosine and sine of its rotation. Mariner writes this form, so
     * the robot uses exactly the values the editor used instead of recomputing them.
     */
    static Shape rect(
            double cx, double cy, double width, double height, double cos, double sin) {
        return new Rect(cx, cy, width, height, cos, sin);
    }

    /** An axis-aligned rectangle between two corners, in either order. */
    static Shape box(double x1, double y1, double x2, double y2) {
        return new Rect(
                (x1 + x2) / 2, (y1 + y2) / 2, Math.abs(x2 - x1), Math.abs(y2 - y1), 1.0, 0.0);
    }

    /** A circle. */
    static Shape circle(double cx, double cy, double radius) {
        return new Circle(cx, cy, radius);
    }

    /**
     * A polygon, which may be concave, tested with the even-odd rule. If the last point repeats
     * the first, it's dropped. A triangle is a three-point polygon.
     *
     * @param xs vertex x coordinates, meters
     * @param ys vertex y coordinates, meters, the same length as {@code xs}
     */
    static Shape polygon(double[] xs, double[] ys) {
        return new Polygon(xs, ys);
    }

    /** A polygon from its vertices. See {@link #polygon(double[], double[])}. */
    static Shape polygon(Translation2d... points) {
        double[] xs = new double[points.length];
        double[] ys = new double[points.length];
        for (int i = 0; i < points.length; i++) {
            xs[i] = points[i].getX();
            ys[i] = points[i].getY();
        }
        return new Polygon(xs, ys);
    }

    /** Cosine and sine of an angle, exact at multiples of 90 degrees. */
    private static double[] cosSin(double degrees) {
        double normalized = ((degrees % 360) + 360) % 360;
        if (normalized == 0) return new double[] {1, 0};
        if (normalized == 90) return new double[] {0, 1};
        if (normalized == 180) return new double[] {-1, 0};
        if (normalized == 270) return new double[] {0, -1};
        double radians = Math.toRadians(degrees);
        return new double[] {Math.cos(radians), Math.sin(radians)};
    }

    /** A rotated rectangle. Use {@link Shape#rect} or {@link Shape#box} to make one. */
    final class Rect implements Shape {
        private final double cx_;
        private final double cy_;
        private final double half_width_;
        private final double half_height_;
        private final double cos_;
        private final double sin_;

        private Rect(double cx, double cy, double width, double height, double cos, double sin) {
            if (!(width > 0) || !(height > 0)) {
                throw new IllegalArgumentException("Rectangle width and height must be positive");
            }
            cx_ = cx;
            cy_ = cy;
            half_width_ = width / 2;
            half_height_ = height / 2;
            cos_ = cos;
            sin_ = sin;
        }

        @Override
        public boolean contains(double x, double y) {
            double dx = x - cx_;
            double dy = y - cy_;
            double local_x = dx * cos_ + dy * sin_;
            double local_y = dy * cos_ - dx * sin_;
            return Math.abs(local_x) <= half_width_ + EDGE_TOLERANCE_METERS
                    && Math.abs(local_y) <= half_height_ + EDGE_TOLERANCE_METERS;
        }

        @Override
        public Translation2d[] outline() {
            double[][] corners = {
                {-half_width_, -half_height_},
                {half_width_, -half_height_},
                {half_width_, half_height_},
                {-half_width_, half_height_},
                {-half_width_, -half_height_},
            };
            Translation2d[] points = new Translation2d[corners.length];
            for (int i = 0; i < corners.length; i++) {
                double lx = corners[i][0];
                double ly = corners[i][1];
                points[i] = new Translation2d(cx_ + lx * cos_ - ly * sin_, cy_ + lx * sin_ + ly * cos_);
            }
            return points;
        }
    }

    /** A circle. Use {@link Shape#circle} to make one. */
    final class Circle implements Shape {
        private final double cx_;
        private final double cy_;
        private final double radius_;

        private Circle(double cx, double cy, double radius) {
            if (!(radius > 0)) {
                throw new IllegalArgumentException("Circle radius must be positive");
            }
            cx_ = cx;
            cy_ = cy;
            radius_ = radius;
        }

        @Override
        public boolean contains(double x, double y) {
            double dx = x - cx_;
            double dy = y - cy_;
            double limit = radius_ + EDGE_TOLERANCE_METERS;
            return dx * dx + dy * dy <= limit * limit;
        }

        @Override
        public Translation2d[] outline() {
            Translation2d[] points = new Translation2d[CIRCLE_OUTLINE_POINTS + 1];
            for (int i = 0; i < CIRCLE_OUTLINE_POINTS; i++) {
                double angle = 2 * Math.PI * i / CIRCLE_OUTLINE_POINTS;
                points[i] =
                        new Translation2d(
                                cx_ + radius_ * Math.cos(angle), cy_ + radius_ * Math.sin(angle));
            }
            points[CIRCLE_OUTLINE_POINTS] = points[0];
            return points;
        }
    }

    /** A polygon. Use {@link Shape#polygon} to make one. */
    final class Polygon implements Shape {
        private final double[] xs_;
        private final double[] ys_;

        private Polygon(double[] xs, double[] ys) {
            if (xs.length != ys.length) {
                throw new IllegalArgumentException("Polygon xs and ys must be the same length");
            }
            int n = xs.length;
            if (n > 1 && xs[0] == xs[n - 1] && ys[0] == ys[n - 1]) {
                n--;
            }
            if (n < 3) {
                throw new IllegalArgumentException("A polygon needs at least 3 points");
            }
            xs_ = Arrays.copyOf(xs, n);
            ys_ = Arrays.copyOf(ys, n);
        }

        @Override
        public boolean contains(double x, double y) {
            boolean inside = false;
            int n = xs_.length;
            for (int i = 0, j = n - 1; i < n; j = i++) {
                double xi = xs_[i];
                double yi = ys_[i];
                double xj = xs_[j];
                double yj = ys_[j];
                double edge_x = xj - xi;
                double edge_y = yj - yi;
                double cross = (x - xi) * edge_y - (y - yi) * edge_x;
                boolean on_line =
                        cross * cross
                                <= EDGE_TOLERANCE_METERS
                                        * EDGE_TOLERANCE_METERS
                                        * (edge_x * edge_x + edge_y * edge_y);
                if (on_line
                        && x >= Math.min(xi, xj) - EDGE_TOLERANCE_METERS
                        && x <= Math.max(xi, xj) + EDGE_TOLERANCE_METERS
                        && y >= Math.min(yi, yj) - EDGE_TOLERANCE_METERS
                        && y <= Math.max(yi, yj) + EDGE_TOLERANCE_METERS) {
                    return true;
                }
                if ((yi > y) != (yj > y) && x < (edge_x * (y - yi)) / edge_y + xi) {
                    inside = !inside;
                }
            }
            return inside;
        }

        @Override
        public Translation2d[] outline() {
            Translation2d[] points = new Translation2d[xs_.length + 1];
            for (int i = 0; i < xs_.length; i++) {
                points[i] = new Translation2d(xs_[i], ys_[i]);
            }
            points[xs_.length] = points[0];
            return points;
        }
    }
}
