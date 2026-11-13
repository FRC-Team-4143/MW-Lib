package com.marswars.regions;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class RegionPublisherTest {
    private NetworkTableInstance nt_;

    @BeforeEach
    public void setUp() {
        FieldRegionTest.setAlliance(Alliance.Blue);
        nt_ = NetworkTableInstance.create();
        RegionPublisher.useInstanceForTesting(nt_);
    }

    @AfterEach
    public void tearDown() {
        RegionPublisher.resetForTesting();
        nt_.close();
        FieldRegionTest.setAlliance(null);
    }

    private StructArraySubscriber<Translation2d> subscribe(String topic) {
        return nt_.getStructArrayTopic(topic, Translation2d.struct).subscribe(new Translation2d[0]);
    }

    @Test
    public void publishesOnRegisterAndAgainOnlyOnChange() {
        FieldRegion flips =
                FieldRegion.builder("FLIPS", FieldRegionTest.ROTATIONAL)
                        .add(Shape.box(1, 1, 2, 2))
                        .subtract(Shape.circle(1.5, 1.5, 0.1))
                        .build();
        FieldRegion fixed =
                FieldRegion.builder("FIXED", FieldRegionTest.ROTATIONAL).fixed().add(Shape.box(1, 1, 2, 2)).build();
        StructArraySubscriber<Translation2d> flipsSub = subscribe("Regions/FLIPS");
        StructArraySubscriber<Translation2d> holeSub = subscribe("Regions/FLIPS/1");
        StructArraySubscriber<Translation2d> fixedSub = subscribe("Regions/FIXED");

        RegionPublisher.register(flips, fixed);
        RegionPublisher.register(flips); // twice is harmless
        assertEquals(1, flipsSub.readQueue().length);
        assertEquals(1, fixedSub.readQueue().length);
        assertTrue(holeSub.get().length > 0, "hole outline on its own topic");
        assertEquals(1, flipsSub.get()[0].getX(), 1e-12);

        RegionPublisher.periodic();
        RegionPublisher.periodic();
        assertEquals(0, flipsSub.readQueue().length, "nothing republished while the alliance holds");

        FieldRegionTest.setAlliance(Alliance.Red);
        RegionPublisher.periodic();
        assertEquals(1, flipsSub.readQueue().length, "republished once on the change");
        assertEquals(0, fixedSub.readQueue().length, "fixed regions are never republished");
        assertEquals(FieldRegionTest.L - 1, flipsSub.get()[0].getX(), 1e-12);
        RegionPublisher.periodic();
        assertEquals(0, flipsSub.readQueue().length);
    }

    @Test
    public void changeBeforeFirstPeriodicIsNotMissed() {
        FieldRegion flips = FieldRegion.builder("EARLY", FieldRegionTest.ROTATIONAL).add(Shape.box(1, 1, 2, 2)).build();
        StructArraySubscriber<Translation2d> sub = subscribe("Regions/EARLY");
        RegionPublisher.register(flips);
        FieldRegionTest.setAlliance(Alliance.Red);
        RegionPublisher.periodic();
        assertEquals(FieldRegionTest.L - 1, sub.get()[0].getX(), 1e-12);
    }
}
