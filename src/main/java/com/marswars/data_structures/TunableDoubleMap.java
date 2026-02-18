package com.marswars.data_structures;

import dev.doglog.DogLog;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import java.util.TreeMap;

/**
 * A wrapper around InterpolatingDoubleTreeMap that automatically generates DogLog tunable entries
 * for the y-values in the map.
 * 
 * <p>This allows runtime tuning of interpolation map values through DogLog without code changes.
 * Each y-value in the map gets a corresponding tunable entry in DogLog with the format:
 * {@code <prefix>/<x_value>}
 * 
 * <p>The tunable values are automatically synchronized when changed via DogLog.
 * 
 * <p>Example usage:
 * <pre>{@code
 * // Create a tunable map for hood angles
 * TunableDoubleMap hoodMap = 
 *     new TunableDoubleMap("LaunchCalculator/HoodAngle");
 * 
 * // Add initial values - these become tunable in DogLog
 * hoodMap.put(1.5, Math.toRadians(20.0));
 * hoodMap.put(2.0, Math.toRadians(25.0));
 * hoodMap.put(3.0, Math.toRadians(30.0));
 * 
 * // Use normally - values will automatically update from DogLog
 * double interpolatedAngle = hoodMap.get(2.5);
 * }</pre>
 */
public class TunableDoubleMap {
    
    private final InterpolatingDoubleTreeMap map_;
    private final String log_prefix_;
    private final TreeMap<Double, Double> initial_values_;
    
    /**
     * Creates a new TunableDoubleMap
     * 
     * @param log_prefix The prefix for DogLog tunable entries (e.g., "Subsystem/MapName")
     */
    public TunableDoubleMap(String log_prefix) {
        this.map_ = new InterpolatingDoubleTreeMap();
        this.log_prefix_ = log_prefix;
        this.initial_values_ = new TreeMap<>();
    }
    
    /**
     * Puts a key-value pair into the map and registers it as a DogLog tunable
     * 
     * <p>The tunable will automatically update the map when changed via DogLog.
     * 
     * @param key The x-value (input)
     * @param value The y-value (output) - becomes tunable in DogLog
     */
    public void put(double key, double value) {
        // Only use DogLog when not in unit test environment
        boolean isUnitTest = false;
        try {
            Class.forName("org.junit.jupiter.api.Test");
            isUnitTest = true;
        } catch (ClassNotFoundException e) {
            // Not a test environment
        }
        
        // Add to map
        map_.put(key, value);
        initial_values_.put(key, value);
        
        // Register tunable entry in DogLog with callback (if not in unit test)
        if (!isUnitTest) {
            String tunable_key = getTunableKey(key);
            DogLog.tunable(tunable_key, value, (val) -> updateValue(key, val));
        }
    }
    
    /**
     * Gets an interpolated value from the map
     * 
     * @param key The x-value to interpolate at
     * @return The interpolated y-value, or NaN if the map is empty or key is out of range
     */
    public double get(double key) {
        Double result = map_.get(key);
        return result != null ? result : Double.NaN;
    }
    
    /**
     * Updates a value in the map (used as callback for DogLog tunables)
     * 
     * @param key The x-value to update
     * @param value The new y-value
     */
    synchronized private void updateValue(double key, double value) {
        map_.put(key, value);
    }
    
    /**
     * Gets the DogLog tunable key for a given x-value
     * 
     * @param key The x-value
     * @return The DogLog key string
     */
    private String getTunableKey(double key) {
        return log_prefix_ + "/" + formatKey(key);
    }
    
    /**
     * Formats a key for display in DogLog
     * 
     * @param key The key to format
     * @return Formatted string
     */
    private String formatKey(double key) {
        // Remove trailing zeros and decimal point if integer
        if (key == Math.floor(key)) {
            return String.valueOf((int) key);
        } else {
            return String.format("%.3f", key).replaceAll("0+$", "").replaceAll("\\.$", "");
        }
    }
}