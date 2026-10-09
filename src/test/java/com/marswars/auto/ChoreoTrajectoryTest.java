package com.marswars.auto;

import static org.junit.jupiter.api.Assertions.*;

import choreo.trajectory.DifferentialSample;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import java.util.List;
import org.junit.jupiter.api.Test;

class ChoreoTrajectoryTest {
  private static final double FIELD_LENGTH = 16.541; // Choreo's flip uses the field dimensions

  private static Trajectory<DifferentialSample> diffTraj() {
    var samples = List.of(
        new DifferentialSample(0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0),
        new DifferentialSample(1, 2, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0),
        new DifferentialSample(2, 3, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0));
    return new Trajectory<>("diff", samples, List.of(), List.of(new EventMarker(1.0, "mid")));
  }

  private static Trajectory<SwerveSample> swerveTraj() {
    var samples = List.of(
        new SwerveSample(0, 1, 1, 0, 1, 0, 0, 0, 0, 0, new double[4], new double[4]),
        new SwerveSample(1, 2, 1, 0, 1, 0, 0, 0, 0, 0, new double[4], new double[4]),
        new SwerveSample(2, 3, 1, 0, 1, 0, 0, 0, 0, 0, new double[4], new double[4]));
    return new Trajectory<>("swerve", samples, List.of(), List.of(new EventMarker(1.0, "mid")));
  }

  @Test
  void differentialKeepsSamplesAndEvents() {
    var traj = new ChoreoTrajectory(diffTraj(), false);
    assertEquals(3, traj.getDifferentialTrajectory().samples().size());
    assertEquals(3, traj.getPoses().length);
    assertEquals(1.0, traj.getEventTimestampMap().get("mid"), 1e-9);
    assertEquals(2.0, traj.getEventPoseMap().get("mid").getX(), 1e-9);
  }

  @Test
  void differentialFlipsForRedAlliance() {
    var traj = new ChoreoTrajectory(diffTraj(), true);
    assertEquals(3, traj.getDifferentialTrajectory().samples().size());
    assertEquals(FIELD_LENGTH - 1.0, traj.getPoses()[0].getX(), 0.1);
    assertEquals(FIELD_LENGTH - 2.0, traj.getEventPoseMap().get("mid").getX(), 0.1);
  }

  @Test
  void getTrajectoryRejectsDifferential() {
    var traj = new ChoreoTrajectory(diffTraj(), false);
    var e = assertThrows(IllegalStateException.class, traj::getTrajectory);
    assertTrue(e.getMessage().contains("DifferentialSample"), e.getMessage());
  }

  @Test
  void getDifferentialTrajectoryRejectsSwerve() {
    var traj = new ChoreoTrajectory(swerveTraj(), false);
    var e = assertThrows(IllegalStateException.class, traj::getDifferentialTrajectory);
    assertTrue(e.getMessage().contains("SwerveSample"), e.getMessage());
  }

  @Test
  void swerveUnchanged() {
    Trajectory<SwerveSample> in = swerveTraj();
    var traj = new ChoreoTrajectory(in, false);
    assertSame(in, traj.getTrajectory());
    assertEquals(3, traj.getPoses().length);
    assertEquals(2.0, traj.getEventPoseMap().get("mid").getX(), 1e-9);
    assertEquals(FIELD_LENGTH - 1.0, new ChoreoTrajectory(in, true).getTrajectory().samples().get(0).x, 0.1);
  }
}
