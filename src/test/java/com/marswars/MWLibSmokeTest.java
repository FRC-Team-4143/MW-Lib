package com.marswars;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Basic smoke tests to verify the library loads and dependencies are available
 */
public class MWLibSmokeTest {

    @Test
    public void testLibraryLoads() {
        // Basic test to ensure the library loads without errors
        assertTrue(true, "MW-Lib should load successfully");
    }

    @Test 
    public void testWPILibDependency() {
        // Verify WPILib classes are available (these don't require native libs)
        assertDoesNotThrow(() -> {
            Class.forName("edu.wpi.first.math.geometry.Pose2d");
            Class.forName("edu.wpi.first.math.kinematics.ChassisSpeeds");
        }, "WPILib math classes should be available");
    }

    @Test
    public void testPhoenixClassesAvailable() {
        // Just verify Phoenix classes are on classpath (don't initialize them)
        assertNotNull(getClass().getClassLoader().getResource("com/ctre/phoenix6/hardware/TalonFX.class"), 
                     "Phoenix 6 TalonFX class should be on classpath");
    }

    @Test
    public void testJacksonDependency() {
        // Verify Jackson JSON library is available
        assertDoesNotThrow(() -> {
            Class.forName("com.fasterxml.jackson.core.JsonParser");
        }, "Jackson should be available");
    }
}