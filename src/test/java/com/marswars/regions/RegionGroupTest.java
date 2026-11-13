package com.marswars.regions;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

public class RegionGroupTest {
    private static final double L = FieldRegionTest.L;
    private static final double W = FieldRegionTest.W;

    @AfterEach
    public void unknown() {
        FieldRegionTest.setAlliance(null);
    }

    @Test
    public void groupBuiltBeforeAnAllianceChangeIsStillRight() {
        // The old HOLD_REGIONS list kept blue's regions after flipRegions(). This can't happen now.
        FieldRegionTest.setAlliance(Alliance.Blue);
        FieldRegion a = FieldRegion.builder("A", FieldRegionTest.ROTATIONAL).add(Shape.box(1, 1, 2, 2)).build();
        FieldRegion b = FieldRegion.builder("B", FieldRegionTest.ROTATIONAL).add(Shape.box(5, 5, 6, 6)).build();
        RegionGroup group = RegionGroup.of("HOLD", a, b);
        assertTrue(group.contains(1.5, 1.5));
        assertTrue(group.contains(5.5, 5.5));
        assertFalse(group.contains(3, 3));

        FieldRegionTest.setAlliance(Alliance.Red);
        assertFalse(group.contains(1.5, 1.5));
        assertTrue(group.contains(L - 1.5, W - 1.5));
        assertTrue(group.contains(L - 5.5, W - 5.5));
    }

    @Test
    public void groupMembersAndValidation() {
        FieldRegion a = FieldRegion.builder("A", FieldRegionTest.ROTATIONAL).add(Shape.circle(1, 1, 1)).build();
        RegionGroup group = RegionGroup.of("G", a);
        assertEquals("G", group.name());
        assertEquals(1, group.regions().size());
        assertThrows(UnsupportedOperationException.class, () -> group.regions().clear());
        assertThrows(IllegalArgumentException.class, () -> RegionGroup.of("EMPTY"));
    }
}
