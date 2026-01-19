package com.marswars.geometry;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class TrajectorylibTest {

    private static final double EPS = 1e-6;

    @Test
    void testVelocityLookupExactRange() {
        Translation3d target = new Translation3d(5.0, 0.0, 1.474);
        Trajectorylib traj = new Trajectorylib(target, true);

        traj.addVelocityPoint(5.0, 8.0);

        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());
        double velocity = traj.getSolution(robotPose).velocity;

        assertEquals(8.0, velocity, EPS);
    }

    @Test
    void testVelocityInterpolation() {
        Translation3d target = new Translation3d(6.0, 0.0, 1.474);
        Trajectorylib traj = new Trajectorylib(target, true);

        traj.addVelocityPoint(4.0, 7.0);
        traj.addVelocityPoint(6.0, 9.0);

        Pose2d robotPose = new Pose2d(1.0, 0.0, new Rotation2d()); // range = 5
        double velocity = traj.getSolution(robotPose).velocity;

        assertEquals(8.0, velocity, EPS);
    }

    @Test
    void testHighArcAngleGreaterThanLowArc() {
        Translation3d target = new Translation3d(5.0, 0.0, 1.474);

        Trajectorylib highArc = new Trajectorylib(target, true);
        Trajectorylib lowArc = new Trajectorylib(target, false);

        highArc.addVelocityPoint(5.0, 9.0);
        lowArc.addVelocityPoint(5.0, 9.0);

        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());

        double highAngle = highArc.getSolution(robotPose).exit_angle;
        double lowAngle = lowArc.getSolution(robotPose).exit_angle;

        assertTrue(highAngle > lowAngle);
    }

    @Test
    void testExitAngleIsFinite() {
        Translation3d target = new Translation3d(4.0, 0.0, 1.474);
        Trajectorylib traj = new Trajectorylib(target, true);

        traj.addVelocityPoint(4.0, 8.5);

        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());
        double angle = traj.getSolution(robotPose).exit_angle;

        assertTrue(Double.isFinite(angle));
    }

    @Test
    void testHeadingAngleStraightAhead() {
        Translation3d target = new Translation3d(5.0, 0.0, 1.474);
        Trajectorylib traj = new Trajectorylib(target, true);

        traj.addVelocityPoint(5.0, 8.0);

        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());
        double heading = traj.getSolution(robotPose).heading_angle;

        assertEquals(0.0, heading, EPS);
    }

    @Test
    void testHeadingAngleQuadrant() {
        Translation3d target = new Translation3d(5.0, 5.0, 1.474);
        Trajectorylib traj = new Trajectorylib(target, true);

        traj.addVelocityPoint(Math.sqrt(50), 9.0);

        Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());
        double heading = traj.getSolution(robotPose).heading_angle;

        assertEquals(Math.atan2(5.0, 5.0), heading, EPS);
    }

    @Test
    void testFullSolutionNotNull() {
        Translation3d target = new Translation3d(6.0, 2.0, 1.474);
        Trajectorylib traj = new Trajectorylib(target, true);

        traj.addVelocityPoint(6.3, 9.5);

        Pose2d robotPose = new Pose2d(0.3, 0.4, new Rotation2d());
        Trajectorylib.TrajectorySol sol = traj.getSolution(robotPose);

        assertNotNull(sol);
        assertTrue(sol.velocity > 0);
        assertTrue(Double.isFinite(sol.exit_angle));
        assertTrue(Double.isFinite(sol.heading_angle));
    }
}

