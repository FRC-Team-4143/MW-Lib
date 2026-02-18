package com.marswars.data_structures;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;

/**
 * Tests for TunableDoubleMap
 * 
 * Note: DogLog.tunable() calls are automatically skipped in unit test environment
 */
public class TunableDoubleMapTest {
    
    private TunableDoubleMap map;
    
    @BeforeEach
    public void setup() {
        map = new TunableDoubleMap("TestMap");
    }
    
    @Test
    public void testBasicPutAndGet() {
        // Add some values
        map.put(1.0, 10.0);
        map.put(2.0, 20.0);
        map.put(3.0, 30.0);
        
        // Test exact matches
        assertEquals(10.0, map.get(1.0), 0.001);
        assertEquals(20.0, map.get(2.0), 0.001);
        assertEquals(30.0, map.get(3.0), 0.001);
    }
    
    @Test
    public void testInterpolation() {
        // Add values for interpolation
        map.put(1.0, 10.0);
        map.put(3.0, 30.0);
        
        // Test interpolation at midpoint
        assertEquals(20.0, map.get(2.0), 0.001);
        
        // Test interpolation at other points
        assertEquals(15.0, map.get(1.5), 0.001);
        assertEquals(25.0, map.get(2.5), 0.001);
    }
    
    @Test
    public void testExtrapolation() {
        // Add values
        map.put(1.0, 10.0);
        map.put(2.0, 20.0);
        
        // Test extrapolation below range (should return first value)
        assertEquals(10.0, map.get(0.5), 0.001);
        
        // Test extrapolation above range (should return last value)
        assertEquals(20.0, map.get(3.0), 0.001);
    }
    
    @Test
    public void testNonLinearInterpolation() {
        // Test with non-linear data (like a shooting trajectory)
        map.put(1.5, Math.toRadians(20.0));
        map.put(2.0, Math.toRadians(25.0));
        map.put(2.5, Math.toRadians(28.0));
        map.put(3.0, Math.toRadians(32.0));
        map.put(4.0, Math.toRadians(40.0));
        
        // Test interpolation at various points
        double angle_at_1_75 = map.get(1.75);
        assertTrue(angle_at_1_75 > Math.toRadians(20.0));
        assertTrue(angle_at_1_75 < Math.toRadians(25.0));
        
        double angle_at_3_5 = map.get(3.5);
        assertTrue(angle_at_3_5 > Math.toRadians(32.0));
        assertTrue(angle_at_3_5 < Math.toRadians(40.0));
    }
    
    @Test
    public void testSingleEntry() {
        // Test with only one entry
        map.put(2.0, 20.0);
        
        // All queries should return the single value
        assertEquals(20.0, map.get(1.0), 0.001);
        assertEquals(20.0, map.get(2.0), 0.001);
        assertEquals(20.0, map.get(3.0), 0.001);
    }
}
