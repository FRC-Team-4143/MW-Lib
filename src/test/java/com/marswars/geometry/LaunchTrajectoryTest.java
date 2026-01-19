package com.marswars.geometry;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class LaunchTrajectoryTest {

    private static final double EPS = 1e-6;

    @Test
    void testVelocityLookupExactRange() {
        Translation3d target = new Translation3d(5.0, 0.0, 1.474);
        LaunchTrajectory traj = new LaunchTrajectory(target, 0.0, true);

        traj.addVelocityPoint(5.0, 15.0); // Increased velocity for realistic trajectory

        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());
        LaunchTrajectory.TrajectorySol sol = traj.getSolution(robotPose);

        assertEquals(15.0, sol.velocity, EPS);
        assertTrue(sol.valid, "Valid trajectory should have valid=true");
    }

    @Test
    void testVelocityInterpolation() {
        Translation3d target = new Translation3d(6.0, 0.0, 1.474);
        LaunchTrajectory traj = new LaunchTrajectory(target, 0.0, true);

        traj.addVelocityPoint(4.0, 12.0); // Increased velocities for realistic trajectories
        traj.addVelocityPoint(6.0, 18.0);

        Pose2d robotPose = new Pose2d(1.0, 0.0, new Rotation2d()); // range = 5
        LaunchTrajectory.TrajectorySol sol = traj.getSolution(robotPose);

        assertEquals(15.0, sol.velocity, EPS); // Updated expected velocity
        assertTrue(sol.valid, "Valid interpolated trajectory should have valid=true");
    }

    @Test
    void testHighArcAngleGreaterThanLowArc() {
        Translation3d target = new Translation3d(5.0, 0.0, 1.474);

        LaunchTrajectory highArc = new LaunchTrajectory(target, 0.0, true);
        LaunchTrajectory lowArc = new LaunchTrajectory(target, 0.0, false);

        highArc.addVelocityPoint(5.0, 15.0); // Increased velocity
        lowArc.addVelocityPoint(5.0, 15.0);

        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());

        double highAngle = highArc.getSolution(robotPose).exit_angle;
        double lowAngle = lowArc.getSolution(robotPose).exit_angle;

        assertTrue(highAngle > lowAngle);
    }

    @Test
    void testExitAngleIsFinite() {
        Translation3d target = new Translation3d(4.0, 0.0, 1.474);
        LaunchTrajectory traj = new LaunchTrajectory(target, 0.0, true);

        traj.addVelocityPoint(4.0, 12.0); // Increased velocity

        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());
        double angle = traj.getSolution(robotPose).exit_angle;

        assertTrue(Double.isFinite(angle));
    }

    @Test
    void testHeadingAngleStraightAhead() {
        Translation3d target = new Translation3d(5.0, 0.0, 1.474);
        LaunchTrajectory traj = new LaunchTrajectory(target, 0.0, true);

        traj.addVelocityPoint(5.0, 15.0); // Increased velocity

        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());
        double heading = traj.getSolution(robotPose).heading_angle;

        assertEquals(0.0, heading, EPS);
    }

    @Test
    void testHeadingAngleQuadrant() {
        Translation3d target = new Translation3d(5.0, 5.0, 1.474);
        LaunchTrajectory traj = new LaunchTrajectory(target, 0.0, true);

        traj.addVelocityPoint(Math.sqrt(50), 16.0); // Increased velocity

        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());
        double heading = traj.getSolution(robotPose).heading_angle;

        assertEquals(Math.atan2(5.0, 5.0), heading, EPS);
    }

    @Test
    void testFullSolutionNotNull() {
        Translation3d target = new Translation3d(6.0, 2.0, 1.474);
        LaunchTrajectory traj = new LaunchTrajectory(target, 0.0, true);

        traj.addVelocityPoint(6.3, 16.0); // Increased velocity

        Pose2d robotPose = new Pose2d(0.3, 0.4, new Rotation2d());
        LaunchTrajectory.TrajectorySol sol = traj.getSolution(robotPose);

        assertNotNull(sol);
        assertTrue(sol.velocity > 0);
        assertTrue(Double.isFinite(sol.exit_angle));
        assertTrue(Double.isFinite(sol.heading_angle));
    }

    @Test
    void testNonZeroLaunchHeightMath() {
        // Target at 2m height, launcher at 1m height -> effective height difference of 1m
        Translation3d target = new Translation3d(5.0, 0.0, 2.0);
        double launchHeight = 1.0;
        LaunchTrajectory traj = new LaunchTrajectory(target, launchHeight, false);

        traj.addVelocityPoint(5.0, 15.0); // Increased velocity

        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());
        LaunchTrajectory.TrajectorySol sol = traj.getSolution(robotPose);

        // Verify the solution is valid
        assertNotNull(sol);
        assertEquals(15.0, sol.velocity, EPS);
        assertTrue(Double.isFinite(sol.exit_angle));
        assertTrue(sol.exit_angle > 0); // Should have positive angle to reach higher target

        // Manual calculation to verify the trajectory math
        // Using kinematic equation: tan(θ) = (v²±√(v⁴-g(gx²+2hv²)))/(gx)
        // where h = target_height - launch_height = 2.0 - 1.0 = 1.0m
        double v = 15.0;
        double x = 5.0;
        double h = 1.0; // effective height difference
        double g = 9.81;
        
        // For low arc (false): tan(θ) = (v²-√(v⁴-g(gx²+2hv²)))/(gx)
        double discriminant = v*v*v*v - g*(g*x*x + 2*h*v*v);
        assertTrue(discriminant >= 0, "Discriminant should be non-negative for valid trajectory");
        
        double expectedAngle = Math.atan2(v*v - Math.sqrt(discriminant), g*x);
        assertEquals(expectedAngle, sol.exit_angle, EPS);
    }

    @Test
    void testFixedAngleShooter() {
        Translation3d target = new Translation3d(5.0, 0.0, 2.0);
        double launchHeight = 1.0;
        double fixedAngle = Math.toRadians(45); // 45 degree shooter
        
        LaunchTrajectory fixedAngleTraj = new LaunchTrajectory(target, launchHeight, fixedAngle);
        
        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());
        LaunchTrajectory.TrajectorySol sol = fixedAngleTraj.getSolution(robotPose);
        
        // Verify the solution properties
        assertNotNull(sol);
        assertTrue(fixedAngleTraj.isFixedAngle());
        assertEquals(fixedAngle, fixedAngleTraj.getFixedAngle(), EPS);
        assertEquals(fixedAngle, sol.exit_angle, EPS);
        assertTrue(Double.isFinite(sol.velocity));
        assertTrue(sol.velocity > 0);
        
        // Manual calculation to verify velocity is correct for 45° angle
        // Using: v² = gx² / (x*sin(2θ) - 2h*cos²(θ))
        double x = 5.0;
        double h = 1.0; // height difference
        double g = 9.81;
        double sin2theta = Math.sin(2 * fixedAngle);
        double cos2theta = Math.cos(fixedAngle) * Math.cos(fixedAngle);
        
        double denominator = x * sin2theta - 2 * h * cos2theta;
        double expectedVelocity = Math.sqrt((g * x * x) / denominator);
        
        assertEquals(expectedVelocity, sol.velocity, EPS);
    }

    @Test
    void testFixedAngleVsVariableAngle() {
        Translation3d target = new Translation3d(4.0, 0.0, 1.5);
        double launchHeight = 0.5;
        
        // Fixed angle at 30 degrees
        double fixedAngle = Math.toRadians(30);
        LaunchTrajectory fixedTraj = new LaunchTrajectory(target, launchHeight, fixedAngle);
        
        // Variable angle (low arc)
        LaunchTrajectory varTraj = new LaunchTrajectory(target, launchHeight, false);
        varTraj.addVelocityPoint(4.0, 12.0);
        
        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());
        
        LaunchTrajectory.TrajectorySol fixedSol = fixedTraj.getSolution(robotPose);
        LaunchTrajectory.TrajectorySol varSol = varTraj.getSolution(robotPose);
        
        // Fixed angle should return exactly the specified angle
        assertEquals(fixedAngle, fixedSol.exit_angle, EPS);
        assertEquals(12.0, varSol.velocity, EPS);
        
        // Both should have the same heading angle
        assertEquals(fixedSol.heading_angle, varSol.heading_angle, EPS);
        
        // Verify both solutions are valid
        assertTrue(Double.isFinite(fixedSol.velocity) && fixedSol.velocity > 0);
        assertTrue(Double.isFinite(varSol.exit_angle));
    }

    @Test
    void testValidFieldPopulation() {
        Translation3d target = new Translation3d(5.0, 0.0, 2.0);
        double launchHeight = 1.0;
        
        // Test valid fixed angle solution
        double validFixedAngle = Math.toRadians(45);
        LaunchTrajectory validFixedTraj = new LaunchTrajectory(target, launchHeight, validFixedAngle);
        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());
        LaunchTrajectory.TrajectorySol validFixedSol = validFixedTraj.getSolution(robotPose);
        
        assertTrue(validFixedSol.valid, "Valid fixed angle trajectory should have valid=true");
        assertTrue(Double.isFinite(validFixedSol.velocity) && validFixedSol.velocity > 0);
        
        // Test invalid fixed angle solution (impossible trajectory)
        double invalidFixedAngle = Math.toRadians(5); // Very shallow angle unlikely to work for upward shot
        LaunchTrajectory invalidFixedTraj = new LaunchTrajectory(target, launchHeight, invalidFixedAngle);
        LaunchTrajectory.TrajectorySol invalidFixedSol = invalidFixedTraj.getSolution(robotPose);
        
        assertFalse(invalidFixedSol.valid, "Invalid fixed angle trajectory should have valid=false");
        
        // Test valid variable angle solution
        LaunchTrajectory validVarTraj = new LaunchTrajectory(target, launchHeight, false);
        validVarTraj.addVelocityPoint(5.0, 15.0); // Increased velocity
        LaunchTrajectory.TrajectorySol validVarSol = validVarTraj.getSolution(robotPose);
        
        assertTrue(validVarSol.valid, "Valid variable angle trajectory should have valid=true");
        assertTrue(Double.isFinite(validVarSol.exit_angle));
        
        // Test invalid variable angle solution (no velocity data)
        LaunchTrajectory invalidVarTraj = new LaunchTrajectory(target, launchHeight, false);
        // Don't add any velocity points - this should result in invalid solution
        LaunchTrajectory.TrajectorySol invalidVarSol = invalidVarTraj.getSolution(robotPose);
        
        assertFalse(invalidVarSol.valid, "Variable angle trajectory with no velocity data should have valid=false");
    }
}

