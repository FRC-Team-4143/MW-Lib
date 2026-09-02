package com.marswars.dashboard;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/**
 * Tests for DashboardChannel's static factory methods. These will fail with
 * UnsupportedOperationException until the TODO(human) factories in DashboardChannel.java are
 * implemented.
 */
public class DashboardChannelTest {

    @Test
    public void bidirectionalIntSetsIntegerAndBidirectional() {
        DashboardChannel channel = DashboardChannel.bidirectionalInt("selected_level");
        assertEquals("selected_level", channel.name());
        assertEquals(DashboardChannel.Type.INTEGER, channel.type());
        assertEquals(DashboardChannel.Direction.BIDIRECTIONAL, channel.direction());
    }

    @Test
    public void outputIntSetsIntegerAndOutputOnly() {
        DashboardChannel channel = DashboardChannel.outputInt("some_output_int");
        assertEquals(DashboardChannel.Type.INTEGER, channel.type());
        assertEquals(DashboardChannel.Direction.OUTPUT_ONLY, channel.direction());
    }

    @Test
    public void bidirectionalBoolSetsBooleanAndBidirectional() {
        DashboardChannel channel = DashboardChannel.bidirectionalBool("coop_state");
        assertEquals(DashboardChannel.Type.BOOLEAN, channel.type());
        assertEquals(DashboardChannel.Direction.BIDIRECTIONAL, channel.direction());
    }

    @Test
    public void outputBoolSetsBooleanAndOutputOnly() {
        DashboardChannel channel = DashboardChannel.outputBool("is_elims");
        assertEquals(DashboardChannel.Type.BOOLEAN, channel.type());
        assertEquals(DashboardChannel.Direction.OUTPUT_ONLY, channel.direction());
    }

    @Test
    public void equalChannelsAreEqualAndHashConsistently() {
        // DashboardBridge uses DashboardChannel as a Map/Set key, so two independently-constructed
        // channels with the same fields must compare equal -- this is free with a record, but worth
        // asserting explicitly since DashboardBridge's correctness depends on it.
        DashboardChannel a = DashboardChannel.bidirectionalInt("l2_state");
        DashboardChannel b = DashboardChannel.bidirectionalInt("l2_state");
        assertEquals(a, b);
        assertEquals(a.hashCode(), b.hashCode());
    }

    @Test
    public void differentNamesAreNotEqual() {
        DashboardChannel a = DashboardChannel.bidirectionalInt("l2_state");
        DashboardChannel b = DashboardChannel.bidirectionalInt("l3_state");
        assertNotEquals(a, b);
    }
}
