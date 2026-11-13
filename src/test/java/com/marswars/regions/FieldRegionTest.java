package com.marswars.regions;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class FieldRegionTest {
    static final double L = 16.541;
    static final double W = 8.069;
    static final FieldGeometry ROTATIONAL = new FieldGeometry(L, W, Symmetry.ROTATIONAL);
    static final FieldGeometry MIRRORED = new FieldGeometry(L, W, Symmetry.MIRRORED);
    static final FieldGeometry ROTATIONAL_CENTER =
            new FieldGeometry(L, W, Symmetry.ROTATIONAL, Origin.CENTER);
    static final FieldGeometry MIRRORED_CENTER =
            new FieldGeometry(L, W, Symmetry.MIRRORED, Origin.CENTER);

    static void setAlliance(Alliance alliance) {
        AllianceState.overrideForTesting(Optional.ofNullable(alliance));
    }

    @BeforeEach
    public void blue() {
        setAlliance(Alliance.Blue);
    }

    @AfterEach
    public void unknown() {
        setAlliance(null);
    }

    /** A small box near blue's right corner (low x, low y). */
    private static FieldRegion corner(FieldGeometry field, boolean flips) {
        FieldRegion.Builder b = FieldRegion.builder("CORNER", field).add(Shape.box(1, 1, 2, 2));
        return (flips ? b.flips() : b.fixed()).build();
    }

    @Test
    public void unionAndHoles() {
        FieldRegion region =
                FieldRegion.builder("ZONE", ROTATIONAL)
                        .add(Shape.box(0, 0, 4, 4), Shape.circle(6, 2, 1))
                        .subtract(Shape.circle(2, 2, 1))
                        .build();
        assertTrue(region.contains(0.5, 0.5));
        assertTrue(region.contains(6.5, 2), "second shape");
        assertFalse(region.contains(2, 2), "hole");
        assertFalse(region.contains(3, 2), "on a hole's edge counts as outside");
        assertTrue(region.contains(3.01, 2));
        assertFalse(region.contains(4.5, 2), "between the shapes");
    }

    @Test
    public void flipsByDefaultAndNeedsAShape() {
        assertEquals(
                AllianceMode.FLIPS,
                FieldRegion.builder("A", ROTATIONAL).add(Shape.circle(0, 0, 1)).build().allianceMode());
        assertThrows(IllegalStateException.class, () -> FieldRegion.builder("A", ROTATIONAL).build());
        assertThrows(IllegalArgumentException.class, () -> FieldRegion.builder(" ", ROTATIONAL));
    }

    @Test
    public void unknownAllianceCountsAsBlue() {
        FieldRegion region = corner(ROTATIONAL, true);
        setAlliance(null);
        assertTrue(region.contains(1.5, 1.5));
        assertFalse(region.contains(L - 1.5, W - 1.5));
    }

    @Test
    public void rotationalBlueCorner() {
        FieldRegion region = corner(ROTATIONAL, true);
        assertTrue(region.contains(1.5, 1.5));
        setAlliance(Alliance.Red);
        assertFalse(region.contains(1.5, 1.5));
        assertTrue(region.contains(L - 1.5, W - 1.5));
        assertFalse(region.contains(L - 1.5, 1.5), "rotational flips y too");
    }

    @Test
    public void mirroredBlueCorner() {
        FieldRegion region = corner(MIRRORED, true);
        setAlliance(Alliance.Red);
        assertTrue(region.contains(L - 1.5, 1.5));
        assertFalse(region.contains(L - 1.5, W - 1.5), "mirrored keeps y");
    }

    @Test
    public void rotationalCenterOrigin() {
        // In center coordinates, (-7, -3) is near one corner; rotating flips both signs.
        FieldRegion region =
                FieldRegion.builder("C", ROTATIONAL_CENTER).add(Shape.box(-7.5, -3.5, -6.5, -2.5)).build();
        assertTrue(region.contains(-7, -3));
        setAlliance(Alliance.Red);
        assertTrue(region.contains(7, 3));
        assertFalse(region.contains(-7, -3));
        assertFalse(region.contains(7, -3));
    }

    @Test
    public void mirroredCenterOrigin() {
        FieldRegion region =
                FieldRegion.builder("C", MIRRORED_CENTER).add(Shape.box(-7.5, -3.5, -6.5, -2.5)).build();
        setAlliance(Alliance.Red);
        assertTrue(region.contains(7, -3));
        assertFalse(region.contains(7, 3));
    }

    @Test
    public void centerOriginMatchesBlueCornerOrigin() {
        // The same physical region in both coordinate systems gives the same answers.
        FieldRegion corner = FieldRegion.builder("A", ROTATIONAL).add(Shape.rect(3, 2, 2, 1, 20)).build();
        FieldRegion center =
                FieldRegion.builder("A", ROTATIONAL_CENTER).add(Shape.rect(L / 2 - 3, W / 2 - 2, 2, 1, 20)).build();
        for (Alliance alliance : new Alliance[] {Alliance.Blue, Alliance.Red}) {
            setAlliance(alliance);
            for (double x = 0; x <= L; x += 0.37) {
                for (double y = 0; y <= W; y += 0.29) {
                    assertEquals(
                            corner.contains(x, y),
                            center.contains(L / 2 - x, W / 2 - y),
                            alliance + " " + x + "," + y);
                }
            }
        }
    }

    @Test
    public void fixedRegionsNeverMove() {
        FieldRegion region = corner(ROTATIONAL, false);
        setAlliance(Alliance.Red);
        assertTrue(region.contains(1.5, 1.5));
        assertFalse(region.contains(L - 1.5, W - 1.5));
    }

    @Test
    public void sameRegionObjectFollowsAllianceChanges() {
        FieldRegion region = corner(ROTATIONAL, true);
        Pose2d pose = new Pose2d(L - 1.5, W - 1.5, Rotation2d.kZero);
        assertFalse(region.contains(pose));
        setAlliance(Alliance.Red);
        assertTrue(region.contains(pose));
        setAlliance(Alliance.Red);
        assertTrue(region.contains(pose), "setting the same alliance twice doesn't toggle");
        setAlliance(Alliance.Blue);
        assertFalse(region.contains(pose));
    }

    @Test
    public void poseTranslationAndXyAgree() {
        FieldRegion region = corner(ROTATIONAL, true);
        assertTrue(region.contains(new Pose2d(1.5, 1.5, Rotation2d.kCW_90deg)));
        assertTrue(region.contains(new Translation2d(1.5, 1.5)));
        assertTrue(region.containsAsDrawn(1.5, 1.5));
    }

    @Test
    public void outlinesFollowTheAlliance() {
        FieldRegion region =
                FieldRegion.builder("Z", ROTATIONAL)
                        .add(Shape.box(1, 1, 2, 2))
                        .subtract(Shape.circle(1.5, 1.5, 0.2))
                        .build();
        List<Translation2d[]> blue = region.outlines();
        assertEquals(2, blue.size(), "one shape, one hole");
        setAlliance(Alliance.Red);
        List<Translation2d[]> red = region.outlines();
        for (int i = 0; i < blue.get(0).length; i++) {
            Translation2d b = blue.get(0)[i];
            Translation2d r = red.get(0)[i];
            assertEquals(L - b.getX(), r.getX(), 1e-12);
            assertEquals(W - b.getY(), r.getY(), 1e-12);
        }
    }
}
