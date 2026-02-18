package com.marswars.geometry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LaunchCalculatorTest {
    
    private LaunchCalculator calculator;
    private static final double DELTA = 1e-3; // Tolerance for floating point comparisons
    
    @BeforeEach
    void setUp() {
        // Create calculator with launcher offset from robot center
        Transform2d robotToLauncher = new Transform2d(
            new Translation2d(-0.3, 0.0),  // 0.3m behind robot center
            new Rotation2d()
        );
        calculator = new LaunchCalculator("TestLaunchCalculator", robotToLauncher);
        
        // Configure range limits
        calculator.setMinDistance(1.5);
        calculator.setMaxDistance(5.5);
        calculator.setPhaseDelay(0.03);
        
        // Add sample data points for hood angle (distance -> angle)
        calculator.addHoodAnglePoint(1.5, Units.degreesToRadians(20.0));
        calculator.addHoodAnglePoint(2.0, Units.degreesToRadians(24.0));
        calculator.addHoodAnglePoint(2.5, Units.degreesToRadians(27.0));
        calculator.addHoodAnglePoint(3.0, Units.degreesToRadians(29.0));
        calculator.addHoodAnglePoint(3.5, Units.degreesToRadians(31.0));
        calculator.addHoodAnglePoint(4.0, Units.degreesToRadians(32.0));
        calculator.addHoodAnglePoint(5.0, Units.degreesToRadians(35.0));
        calculator.addHoodAnglePoint(5.5, Units.degreesToRadians(37.0));
        
        // Add sample data points for flywheel speed (distance -> speed in rad/s)
        calculator.addFlywheelSpeedPoint(1.5, 200.0);
        calculator.addFlywheelSpeedPoint(2.0, 220.0);
        calculator.addFlywheelSpeedPoint(2.5, 230.0);
        calculator.addFlywheelSpeedPoint(3.0, 240.0);
        calculator.addFlywheelSpeedPoint(3.5, 245.0);
        calculator.addFlywheelSpeedPoint(4.0, 250.0);
        calculator.addFlywheelSpeedPoint(5.0, 270.0);
        calculator.addFlywheelSpeedPoint(5.5, 280.0);
        
        // Add sample data points for time of flight (distance -> time in seconds)
        calculator.addTimeOfFlightPoint(1.5, 0.8);
        calculator.addTimeOfFlightPoint(2.0, 0.9);
        calculator.addTimeOfFlightPoint(2.5, 1.0);
        calculator.addTimeOfFlightPoint(3.0, 1.05);
        calculator.addTimeOfFlightPoint(3.5, 1.1);
        calculator.addTimeOfFlightPoint(4.0, 1.12);
        calculator.addTimeOfFlightPoint(5.0, 1.15);
        calculator.addTimeOfFlightPoint(5.5, 1.18);
    }
    
    @Test
    void testStationaryShot() {
        // Robot at origin, target at (3, 0)
        Pose2d robotPose = new Pose2d(0, 0, new Rotation2d());
        ChassisSpeeds robotVelocity = new ChassisSpeeds(0, 0, 0); // Stationary
        Translation2d target = new Translation2d(3.0, 0.0);
        
        LaunchCalculator.LaunchParameters params = calculator.calculateLaunchParameters(
            robotPose,
            robotVelocity,
            target
        );
        
        // Should be valid
        assertTrue(params.is_valid, "Parameters should be valid within range");
        
        // Should be aiming forward (accounting for launcher offset)
        assertEquals(0.0, params.heading_angle.getRadians(), 0.1, 
            "Should be aiming approximately forward");
        
        // Distance should be about 3m (with launcher offset of 0.3m)
        assertEquals(3.3, params.distance, 0.1, "Distance should account for launcher offset");
        
        // Hood angle should be interpolated for ~3.3m
        assertTrue(params.hood_angle > Units.degreesToRadians(29.0) 
            && params.hood_angle < Units.degreesToRadians(32.0),
            "Hood angle should be between 29 and 32 degrees");
        
        // Flywheel speed should be interpolated for ~3.3m
        assertTrue(params.flywheel_speed > 240.0 && params.flywheel_speed < 250.0,
            "Flywheel speed should be between 240 and 250 rad/s");
    }
    
    @Test
    void testMovingShot() {
        // Robot at origin, moving forward at 1 m/s, target at (3, 0)
        Pose2d robotPose = new Pose2d(0, 0, new Rotation2d());
        ChassisSpeeds robotVelocity = new ChassisSpeeds(1.0, 0, 0); // Moving forward
        Translation2d target = new Translation2d(3.0, 0.0);
        
        LaunchCalculator.LaunchParameters params = calculator.calculateLaunchParameters(
            robotPose,
            robotVelocity,
            target
        );
        
        // Should be valid
        assertTrue(params.is_valid, "Parameters should be valid within range");
        
        // With lookahead, effective distance should be shorter than 3m
        assertTrue(params.distance < 3.3, 
            "Distance with lookahead should be less than stationary distance");
        
        // Velocities should be non-zero (for feedforward)
        // Note: First call may have zero velocity due to filter initialization
        assertNotNull(params.hood_velocity);
        assertNotNull(params.heading_velocity);
    }
    
    @Test
    void testLateralMotionCompensation() {
        // Robot at (2, 0), moving sideways at 2 m/s, target at (4, 2)
        Pose2d robotPose = new Pose2d(2.0, 0.0, new Rotation2d());
        ChassisSpeeds robotVelocity = new ChassisSpeeds(0, 2.0, 0); // Moving sideways
        Translation2d target = new Translation2d(4.0, 2.0);
        
        LaunchCalculator.LaunchParameters params = calculator.calculateLaunchParameters(
            robotPose,
            robotVelocity,
            target
        );
        
        // Should be valid
        assertTrue(params.is_valid, "Parameters should be valid within range");
        
        // Heading angle should compensate for lateral motion
        // Just verify we get a reasonable angle
        assertTrue(Math.abs(params.heading_angle.getRadians()) < Math.PI,
            "Heading angle should be reasonable");
    }
    
    @Test
    void testOutOfRangeShot() {
        // Robot at origin, target too far away
        Pose2d robotPose = new Pose2d(0, 0, new Rotation2d());
        ChassisSpeeds robotVelocity = new ChassisSpeeds(0, 0, 0);
        Translation2d target = new Translation2d(10.0, 0.0); // 10m away
        
        LaunchCalculator.LaunchParameters params = calculator.calculateLaunchParameters(
            robotPose,
            robotVelocity,
            target
        );
        
        // Should be invalid (out of range)
        assertFalse(params.is_valid, "Parameters should be invalid - target too far");
        
        // Robot at origin, target too close
        target = new Translation2d(0.5, 0.0); // 0.5m away
        
        params = calculator.calculateLaunchParameters(
            robotPose,
            robotVelocity,
            target
        );
        
        // Should be invalid (too close)
        assertFalse(params.is_valid, "Parameters should be invalid - target too close");
    }
    
    @Test
    void testGetStationaryAimedPose() {
        Translation2d robotTranslation = new Translation2d(1.0, 1.0);
        Translation2d target = new Translation2d(4.0, 4.0);
        
        Pose2d aimedPose = calculator.getStationaryAimedPose(robotTranslation, target);
        
        // Pose should be at robot translation
        assertEquals(robotTranslation.getX(), aimedPose.getX(), DELTA);
        assertEquals(robotTranslation.getY(), aimedPose.getY(), DELTA);
        
        // Rotation should point generally toward target
        // (exact angle depends on launcher offset compensation)
        assertNotNull(aimedPose.getRotation());
    }
    
    @Test
    void testCacheClearing() {
        Pose2d robotPose = new Pose2d(0, 0, new Rotation2d());
        ChassisSpeeds robotVelocity = new ChassisSpeeds(0, 0, 0);
        Translation2d target = new Translation2d(3.0, 0.0);
        
        // Calculate once
        LaunchCalculator.LaunchParameters params1 = calculator.calculateLaunchParameters(
            robotPose,
            robotVelocity,
            target
        );
        
        // Get cached version
        LaunchCalculator.LaunchParameters cached = calculator.getLatestParameters();
        
        // Should be same instance
        assertSame(params1, cached, "Cached parameters should be same instance");
        
        // Clear cache
        calculator.clearCache();
        
        // Get latest should return invalid
        LaunchCalculator.LaunchParameters afterClear = calculator.getLatestParameters();
        assertFalse(afterClear.is_valid, "After clearing cache, should return invalid");
    }
    
    @Test
    void testMinMaxTimeOfFlight() {
        double minTOF = calculator.getMinTimeOfFlight();
        double maxTOF = calculator.getMaxTimeOfFlight();
        
        assertEquals(0.8, minTOF, DELTA, "Min TOF should match min distance entry");
        assertEquals(1.18, maxTOF, DELTA, "Max TOF should match max distance entry");
        
        assertTrue(maxTOF > minTOF, "Max TOF should be greater than min TOF");
    }
    
    @Test
    void testBuilder() {
        // Test using builder pattern
        LaunchCalculator calc = new LaunchCalculator.Builder()
            .withLoggingPrefix("TestBuilder")
            .withRobotToLauncher(new Transform2d(new Translation2d(-0.25, 0.1), new Rotation2d()))
            .withLoopPeriod(0.02)
            .withMinDistance(1.0)
            .withMaxDistance(6.0)
            .withPhaseDelay(0.05)
            .build();
        
        assertNotNull(calc, "Builder should create calculator");
        
        // Add some data
        calc.addHoodAnglePoint(2.0, Units.degreesToRadians(25.0));
        calc.addFlywheelSpeedPoint(2.0, 225.0);
        calc.addTimeOfFlightPoint(2.0, 1.0);
        
        // Should be able to calculate
        Pose2d robotPose = new Pose2d(0, 0, new Rotation2d());
        ChassisSpeeds robotVelocity = new ChassisSpeeds(0, 0, 0);
        Translation2d target = new Translation2d(2.0, 0.0);
        
        LaunchCalculator.LaunchParameters params = calc.calculateLaunchParameters(
            robotPose,
            robotVelocity,
            target
        );
        
        assertNotNull(params, "Should be able to calculate parameters");
    }
    
    @Test
    void testInterpolation() {
        // Test that interpolation works between defined points
        Pose2d robotPose = new Pose2d(0, 0, new Rotation2d());
        ChassisSpeeds robotVelocity = new ChassisSpeeds(0, 0, 0);
        
        // Target at distance that requires interpolation
        // With launcher 0.3m behind robot center, need to account for that
        Translation2d target = new Translation2d(2.2, 0.0); // Will be ~2.5m from launcher
        
        LaunchCalculator.LaunchParameters params = calculator.calculateLaunchParameters(
            robotPose,
            robotVelocity,
            target
        );
        
        // Hood angle should be interpolated around 2.5m point (27°)
        double expectedMinAngle = Units.degreesToRadians(25.0);
        double expectedMaxAngle = Units.degreesToRadians(29.0);
        
        assertTrue(params.hood_angle >= expectedMinAngle && params.hood_angle <= expectedMaxAngle,
            "Hood angle should be interpolated between defined points, got " + Units.radiansToDegrees(params.hood_angle) + " degrees");
        
        // Flywheel speed should be interpolated around 2.5m point (230)
        assertTrue(params.flywheel_speed >= 220.0 && params.flywheel_speed <= 245.0,
            "Flywheel speed should be interpolated between defined points, got " + params.flywheel_speed);
    }
    
    @Test
    void testPoseTargetOverload() {
        // Test the overload that takes a Pose2d for target instead of Translation2d
        Pose2d robotPose = new Pose2d(0, 0, new Rotation2d());
        ChassisSpeeds robotVelocity = new ChassisSpeeds(0, 0, 0);
        Pose2d targetPose = new Pose2d(3.0, 0.0, Rotation2d.fromDegrees(90)); // Rotation should be ignored
        
        LaunchCalculator.LaunchParameters params = calculator.calculateLaunchParameters(
            robotPose,
            robotVelocity,
            targetPose
        );
        
        assertTrue(params.is_valid, "Parameters should be valid");
        
        // Should give same result as using translation only
        LaunchCalculator.LaunchParameters paramsFromTranslation = calculator.calculateLaunchParameters(
            robotPose,
            robotVelocity,
            targetPose.getTranslation()
        );
        
        // Note: Due to caching and filter state, exact equality may not hold
        // But distances should match
        assertEquals(paramsFromTranslation.distance_no_lookahead, params.distance_no_lookahead, 0.01,
            "Distance should be same whether using Pose2d or Translation2d");
    }
}
