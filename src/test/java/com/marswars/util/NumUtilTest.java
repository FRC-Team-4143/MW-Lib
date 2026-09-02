package com.marswars.util;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for NumUtil utility class
 */
public class NumUtilTest {

    @Test
    public void testNumUtilExists() {
        // Basic test to ensure NumUtil class exists and loads
        assertDoesNotThrow(() -> {
            Class.forName("com.marswars.util.NumUtil");
        }, "NumUtil class should exist");
    }

    @Test
    public void packBitsMatchesHandComputedExample() {
        // bit 0 = true, bit 1 = false, bit 2 = true, bit 3 = true -> 0b1101 = 13
        assertEquals(13, NumUtil.packBits(new boolean[] {true, false, true, true}));
    }

    @Test
    public void unpackBitsMatchesHandComputedExample() {
        assertArrayEquals(new boolean[] {true, false, true, true}, NumUtil.unpackBits(13, 4));
    }

    @Test
    public void packBitsOfEmptyArrayIsZero() {
        assertEquals(0, NumUtil.packBits(new boolean[0]));
    }

    @Test
    public void unpackBitsOfZeroIsAllFalse() {
        assertArrayEquals(new boolean[] {false, false, false}, NumUtil.unpackBits(0, 3));
    }

    @Test
    public void packThenUnpackRoundTrips() {
        boolean[] bits = {true, true, false, true, false, false, true};
        assertArrayEquals(bits, NumUtil.unpackBits(NumUtil.packBits(bits), bits.length));
    }
}