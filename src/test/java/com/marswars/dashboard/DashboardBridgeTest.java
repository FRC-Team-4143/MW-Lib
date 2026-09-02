package com.marswars.dashboard;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for DashboardBridge's pure NetworkTables plumbing -- no HTTP server, no filesystem, no
 * robot hardware needed. Each test gets its own scratch NetworkTableInstance so tests can't see
 * each other's topics.
 */
public class DashboardBridgeTest {

    private static final DashboardBridge.Config CONFIG =
            new DashboardBridge.Config("/Test/ToRobot", "/Test/ToDashboard", 5802, "unused");

    // Constructed directly (not via the TODO(human) factories in DashboardChannel) so these tests
    // don't depend on that other piece being implemented yet.
    private static final DashboardChannel BIDI_INT =
            new DashboardChannel(
                    "bidi_int", DashboardChannel.Type.INTEGER, DashboardChannel.Direction.BIDIRECTIONAL);
    private static final DashboardChannel BIDI_BOOL =
            new DashboardChannel(
                    "bidi_bool",
                    DashboardChannel.Type.BOOLEAN,
                    DashboardChannel.Direction.BIDIRECTIONAL);
    private static final DashboardChannel OUTPUT_BOOL =
            new DashboardChannel(
                    "output_bool", DashboardChannel.Type.BOOLEAN, DashboardChannel.Direction.OUTPUT_ONLY);

    private NetworkTableInstance nt_;

    @BeforeEach
    public void setUp() {
        nt_ = NetworkTableInstance.create();
    }

    @AfterEach
    public void tearDown() {
        nt_.close();
    }

    private DashboardBridge newBridge(DashboardChannel... channels) {
        return new DashboardBridge(nt_, CONFIG, List.of(channels));
    }

    @Test
    public void inboundIntRoundTripsThroughReadInputs() {
        DashboardBridge bridge = newBridge(BIDI_INT);
        IntegerPublisher dashboardSide =
                nt_.getTable(CONFIG.toRobotTable())
                        .getIntegerTopic(BIDI_INT.name())
                        .publish(PubSubOption.keepDuplicates(true));

        dashboardSide.set(7);
        nt_.flushLocal();
        bridge.readInputs();

        assertEquals(7, bridge.getIntIfChanged(BIDI_INT).getAsInt());
        assertEquals(7, bridge.getInt(BIDI_INT));
    }

    @Test
    public void inboundBoolRoundTripsThroughReadInputs() {
        DashboardBridge bridge = newBridge(BIDI_BOOL);
        var dashboardSide =
                nt_.getTable(CONFIG.toRobotTable())
                        .getBooleanTopic(BIDI_BOOL.name())
                        .publish(PubSubOption.keepDuplicates(true));

        dashboardSide.set(true);
        nt_.flushLocal();
        bridge.readInputs();

        assertTrue(bridge.getBoolIfChanged(BIDI_BOOL).orElseThrow());
        assertTrue(bridge.getBool(BIDI_BOOL));
    }

    @Test
    public void quietTickReportsNoChangeButKeepsLastValue() {
        DashboardBridge bridge = newBridge(BIDI_INT);
        IntegerPublisher dashboardSide =
                nt_.getTable(CONFIG.toRobotTable())
                        .getIntegerTopic(BIDI_INT.name())
                        .publish(PubSubOption.keepDuplicates(true));

        dashboardSide.set(3);
        nt_.flushLocal();
        bridge.readInputs();
        assertTrue(bridge.getIntIfChanged(BIDI_INT).isPresent());

        // No new publish this time -- readInputs() again should see nothing new.
        bridge.readInputs();
        assertTrue(bridge.getIntIfChanged(BIDI_INT).isEmpty());
        assertEquals(3, bridge.getInt(BIDI_INT)); // but the last known value is unchanged
    }

    @Test
    public void outputOnlyChannelHasNoInboundSubscriber() {
        DashboardBridge bridge = newBridge(OUTPUT_BOOL);
        assertThrows(IllegalArgumentException.class, () -> bridge.getBoolIfChanged(OUTPUT_BOOL));
    }

    @Test
    public void setSuppressesRedundantPublishes() {
        DashboardBridge bridge = newBridge(BIDI_INT);
        IntegerSubscriber dashboardSide =
                nt_.getTable(CONFIG.toDashboardTable())
                        .getIntegerTopic(BIDI_INT.name())
                        .subscribe(-1, PubSubOption.keepDuplicates(true));

        bridge.set(BIDI_INT, 5);
        nt_.flushLocal();
        bridge.set(BIDI_INT, 5); // same value again
        nt_.flushLocal();

        assertEquals(1, dashboardSide.readQueue().length, "identical value should publish once");

        bridge.set(BIDI_INT, 6);
        nt_.flushLocal();
        assertEquals(1, dashboardSide.readQueue().length, "changed value should publish");
    }

    @Test
    public void wrongTypeAccessorThrows() {
        DashboardBridge bridge = newBridge(BIDI_INT);
        assertThrows(IllegalArgumentException.class, () -> bridge.getBool(BIDI_INT));
        assertThrows(IllegalArgumentException.class, () -> bridge.set(BIDI_INT, true));
    }

    @Test
    public void unregisteredChannelThrows() {
        DashboardBridge bridge = newBridge(BIDI_INT);
        assertThrows(IllegalArgumentException.class, () -> bridge.getInt(BIDI_BOOL));
    }

    @Test
    public void duplicateChannelNameInConstructorThrows() {
        assertThrows(
                IllegalArgumentException.class,
                () ->
                        new DashboardBridge(
                                nt_,
                                CONFIG,
                                List.of(
                                        BIDI_INT,
                                        new DashboardChannel(
                                                BIDI_INT.name(),
                                                DashboardChannel.Type.BOOLEAN,
                                                DashboardChannel.Direction.BIDIRECTIONAL))));
    }

    @Test
    public void setStillPublishesWhenEchoingAJustReceivedInboundValue() {
        DashboardBridge bridge = newBridge(BIDI_INT);

        // Subscribe to ToDashboard *before* anything happens, exactly like
        // setSuppressesRedundantPublishes does, so we can observe what the bridge actually sends.
        IntegerSubscriber dashboardSide =
                nt_.getTable(CONFIG.toDashboardTable())
                        .getIntegerTopic(BIDI_INT.name())
                        .subscribe(-1, PubSubOption.keepDuplicates(true));

        // 1. Simulate the dashboard publishing a value on ToRobot.
        IntegerPublisher fromDashboard =
                nt_.getTable(CONFIG.toRobotTable())
                        .getIntegerTopic(BIDI_INT.name())
                        .publish(PubSubOption.keepDuplicates(true));
        fromDashboard.set(9);
        nt_.flushLocal();

        // 2. The bridge picks it up -- current_int_values_ now holds 9.
        bridge.readInputs();
        assertEquals(9, bridge.getIntIfChanged(BIDI_INT).getAsInt());

        // 3. Consumer code echoes the accepted value straight back out.
        bridge.set(BIDI_INT, 9);
        nt_.flushLocal();

        // 4. If the dirty-check wrongly compared against current_int_values_ instead of
        // last_published_int_, it would see "9 == 9" and skip the publish, so the dashboard would
        // never actually be told the robot accepted its own tap. This assertion fails if that
        // bug is reintroduced.
        assertEquals(
                1,
                dashboardSide.readQueue().length,
                "echoing a just-received value must still publish, not be suppressed as unchanged");
    }
}
