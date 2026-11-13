package com.marswars.regions;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import java.util.ArrayList;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

/**
 * Publishes region outlines to NetworkTables, for AdvantageScope's 2D field view. Outlines are sent
 * once when regions are registered, and again only when the alliance changes, so a steady loop
 * costs one comparison.
 *
 * <p>A region's first outline goes to {@code Regions/<name>}, and any more (extra shapes and holes)
 * to {@code Regions/<name>/1}, {@code Regions/<name>/2} and so on. {@link
 * com.marswars.subsystem.SubsystemManager} calls {@link #periodic()} every loop; robots that don't
 * use it should call it from {@code robotPeriodic()}.
 */
public final class RegionPublisher {
    /** The NetworkTables folder outlines are published under. */
    public static final String TABLE = "Regions";

    private static final Map<FieldRegion, List<StructArrayPublisher<Translation2d>>> publishers_ =
            new IdentityHashMap<>();
    private static NetworkTableInstance nt_ = null;
    private static long published_change_count_ = Long.MIN_VALUE;

    private RegionPublisher() {}

    /** Starts publishing these regions' outlines. Registering a region twice does nothing. */
    public static synchronized void register(Collection<FieldRegion> regions) {
        if (publishers_.isEmpty()) {
            published_change_count_ = AllianceState.changeCount();
        }
        for (FieldRegion region : regions) {
            if (!publishers_.containsKey(region)) {
                publishers_.put(region, new ArrayList<>());
                publish(region);
            }
        }
    }

    /** Starts publishing these regions' outlines. Registering a region twice does nothing. */
    public static void register(FieldRegion... regions) {
        register(List.of(regions));
    }

    /** Republishes flipping regions if the alliance has changed since they were last published. */
    public static synchronized void periodic() {
        if (publishers_.isEmpty()) {
            return;
        }
        long change_count = AllianceState.changeCount();
        if (change_count == published_change_count_) {
            return;
        }
        published_change_count_ = change_count;
        for (FieldRegion region : publishers_.keySet()) {
            if (region.allianceMode() == AllianceMode.FLIPS) {
                publish(region);
            }
        }
    }

    private static void publish(FieldRegion region) {
        List<StructArrayPublisher<Translation2d>> publishers = publishers_.get(region);
        List<Translation2d[]> outlines = region.outlines();
        while (publishers.size() < outlines.size()) {
            int index = publishers.size();
            String topic = TABLE + "/" + region.name() + (index == 0 ? "" : "/" + index);
            publishers.add(instance().getStructArrayTopic(topic, Translation2d.struct).publish());
        }
        for (int i = 0; i < outlines.size(); i++) {
            publishers.get(i).set(outlines.get(i));
        }
    }

    private static NetworkTableInstance instance() {
        if (nt_ == null) {
            nt_ = NetworkTableInstance.getDefault();
        }
        return nt_;
    }

    /** Publishes to this instance instead of the default one. For tests. */
    static synchronized void useInstanceForTesting(NetworkTableInstance instance) {
        resetForTesting();
        nt_ = instance;
    }

    /** Closes every publisher and forgets every region. For tests. */
    static synchronized void resetForTesting() {
        for (List<StructArrayPublisher<Translation2d>> publishers : publishers_.values()) {
            publishers.forEach(StructArrayPublisher::close);
        }
        publishers_.clear();
        published_change_count_ = Long.MIN_VALUE;
        nt_ = null;
    }
}
