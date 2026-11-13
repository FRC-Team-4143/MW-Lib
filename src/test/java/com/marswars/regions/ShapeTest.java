package com.marswars.regions;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

public class ShapeTest {
    private static final double TINY = 1e-12;
    private static final double OUT = 1e-6;

    @Test
    public void boxIncludesEdgesAndCorners() {
        Shape box = Shape.box(1, 1, 3, 2);
        assertTrue(box.contains(2, 1.5));
        assertTrue(box.contains(1, 1), "corner");
        assertTrue(box.contains(3, 1.5), "right edge");
        assertTrue(box.contains(2, 2 + TINY), "within tolerance");
        assertFalse(box.contains(2, 2 + OUT));
        assertFalse(box.contains(0.5, 1.5));
    }

    @Test
    public void boxCornersInEitherOrder() {
        Shape a = Shape.box(3, 2, 1, 1);
        assertTrue(a.contains(2, 1.5));
        assertFalse(a.contains(3.5, 1.5));
    }

    @Test
    public void rectRotatedNinetyDegreesSwapsWidthAndHeight() {
        Shape rect = Shape.rect(0, 0, 4, 1, 90);
        assertTrue(rect.contains(0, 2), "end of the long side, now along y");
        assertFalse(rect.contains(2, 0), "would be inside if it weren't rotated");
        assertTrue(rect.contains(0.5, 0), "short side edge");
        assertFalse(rect.contains(0.5 + OUT, 0));
    }

    @Test
    public void rectRotatedFortyFiveDegrees() {
        Shape rect = Shape.rect(0, 0, 2, 2, 45);
        double corner = Math.sqrt(2);
        assertTrue(rect.contains(corner - OUT, 0), "corner now points along +x");
        assertFalse(rect.contains(corner + OUT, 0));
        assertFalse(rect.contains(0.9, 0.9), "inside the unrotated square, outside the rotated one");
    }

    @Test
    public void rectFullTurnMatchesUnrotated() {
        Shape a = Shape.rect(1, 1, 2, 1, 0);
        Shape b = Shape.rect(1, 1, 2, 1, 360);
        Shape c = Shape.rect(1, 1, 2, 1, -180);
        for (double[] p : new double[][] {{2, 1.5}, {2 + OUT, 1}, {0, 0.5}, {1, 1.5 + OUT}}) {
            assertEquals(a.contains(p[0], p[1]), b.contains(p[0], p[1]));
            assertEquals(a.contains(p[0], p[1]), c.contains(p[0], p[1]));
        }
    }

    @Test
    public void rectFromCosSinMatchesDegrees() {
        double r = Math.toRadians(30);
        Shape a = Shape.rect(5, 4, 2, 1, 30);
        Shape b = Shape.rect(5, 4, 2, 1, Math.cos(r), Math.sin(r));
        for (double x = 3.5; x <= 6.5; x += 0.25) {
            for (double y = 2.5; y <= 5.5; y += 0.25) {
                assertEquals(a.contains(x, y), b.contains(x, y), x + "," + y);
            }
        }
    }

    @Test
    public void circleIncludesEdge() {
        Shape circle = Shape.circle(2, 2, 1);
        assertTrue(circle.contains(2, 2));
        assertTrue(circle.contains(3, 2), "edge");
        assertTrue(circle.contains(3 + TINY, 2));
        assertFalse(circle.contains(3 + OUT, 2));
        assertFalse(circle.contains(2.8, 2.8));
    }

    @Test
    public void concavePolygonUsesEvenOdd() {
        // A "U": the notch between the arms is outside.
        Shape u = Shape.polygon(new double[] {0, 3, 3, 2, 2, 1, 1, 0}, new double[] {0, 0, 3, 3, 1, 1, 3, 3});
        assertTrue(u.contains(0.5, 2), "left arm");
        assertTrue(u.contains(2.5, 2), "right arm");
        assertTrue(u.contains(1.5, 0.5), "base");
        assertFalse(u.contains(1.5, 2), "notch");
        assertTrue(u.contains(1.5, 1), "notch floor is an edge");
        assertTrue(u.contains(1, 2), "notch wall is an edge");
    }

    @Test
    public void polygonVerticesAndEdgesAreInside() {
        Shape tri = Shape.polygon(new Translation2d(0, 0), new Translation2d(4, 0), new Translation2d(0, 4));
        assertTrue(tri.contains(0, 0));
        assertTrue(tri.contains(4, 0));
        assertTrue(tri.contains(2, 2), "hypotenuse");
        assertFalse(tri.contains(2 + OUT, 2 + OUT));
        assertTrue(tri.contains(1, 1));
    }

    @Test
    public void polygonDropsRepeatedClosingPoint() {
        Shape closed = Shape.polygon(
                new Translation2d(0, 0), new Translation2d(2, 0), new Translation2d(2, 2), new Translation2d(0, 0));
        assertEquals(4, closed.outline().length, "three corners plus the closing point");
        assertTrue(closed.contains(1.5, 0.5));
    }

    @Test
    public void outlinesAreClosedLoops() {
        for (Shape shape : new Shape[] {Shape.box(0, 0, 1, 1), Shape.circle(0, 0, 1), Shape.rect(0, 0, 2, 1, 30)}) {
            Translation2d[] outline = shape.outline();
            assertEquals(outline[0], outline[outline.length - 1]);
            for (Translation2d p : outline) {
                assertTrue(shape.contains(p.getX(), p.getY()), "outline points lie on the edge");
            }
        }
        Translation2d[] rotated = Shape.rect(0, 0, 4, 2, 90).outline();
        for (Translation2d p : rotated) {
            assertEquals(1, Math.abs(p.getX()), 1e-12);
            assertEquals(2, Math.abs(p.getY()), 1e-12);
        }
    }

    @Test
    public void rejectsBadShapes() {
        assertThrows(IllegalArgumentException.class, () -> Shape.circle(0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> Shape.rect(0, 0, -1, 1, 0));
        assertThrows(IllegalArgumentException.class, () -> Shape.box(0, 0, 0, 1));
        assertThrows(IllegalArgumentException.class, () -> Shape.polygon(new double[] {0, 1}, new double[] {0, 1}));
        assertThrows(IllegalArgumentException.class, () -> Shape.polygon(new double[] {0, 1, 2}, new double[] {0, 1}));
    }
}
