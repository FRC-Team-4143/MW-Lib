package com.marswars.auto;

import static org.junit.jupiter.api.Assertions.*;

import choreo.trajectory.DifferentialSample;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import org.junit.jupiter.api.Test;

class ChoreoTrajectoryTest {
  private static DifferentialSample dsample(double t, double x) {
    return new DifferentialSample(t, x, 1.0, 0.0, 1.0, 1.0, 0.0, 0, 0, 0, 0, 0);
  }

  private static SwerveSample ssample(double t, double x) {
    return new SwerveSample(t, x, 1.0, 0.0, 1.0, 0, 0, 0, 0, 0, new double[4], new double[4]);
  }

  private static Trajectory<DifferentialSample> diffTraj() {
    return new Trajectory<>(
        "diff",
        List.of(dsample(0, 0), dsample(1, 1), dsample(2, 2)),
        List.of(),
        List.of(new EventMarker(1.0, "mid")));
  }

  private static Trajectory<SwerveSample> swerveTraj() {
    return new Trajectory<>(
        "swerve",
        List.of(ssample(0, 0), ssample(1, 1), ssample(2, 2)),
        List.of(),
        List.of(new EventMarker(1.0, "mid")));
  }

  @Test
  void differentialWraps() {
    ChoreoTrajectory t = ChoreoTrajectory.of(diffTraj(), false);
    assertTrue(t.isDifferential());
    assertFalse(t.isSwerve());
    assertEquals(3, t.getPoses().length);
    assertEquals(0.0, t.getInitialPose().get().getX(), 1e-9);
    assertEquals(2.0, t.getFinalPose().get().getX(), 1e-9);
    assertEquals(0.5, t.samplePoseAt(0.5).get().getX(), 1e-9);
    assertEquals(2.0, t.getTotalTime(), 1e-9);
    assertEquals(1.0, t.getEventTimestampMap().get("mid"), 1e-9);
    assertEquals(1.0, t.getEventPoseMap().get("mid").getX(), 1e-9);
    assertEquals(2, t.getDifferentialTrajectory().samples().get(2).getTimestamp(), 1e-9);
    assertThrows(IllegalStateException.class, t::getTrajectory);
  }

  @Test
  void differentialFlipsForRed() {
    ChoreoTrajectory t = ChoreoTrajectory.of(diffTraj(), true);
    assertNotEquals(0.0, t.getInitialPose().get().getX(), 1e-6);
  }

  @Test
  void swerveApiUnchanged() {
    ChoreoTrajectory t = new ChoreoTrajectory(swerveTraj(), false);
    Trajectory<SwerveSample> traj = t.getTrajectory();
    assertEquals(3, traj.samples().size());
    assertTrue(t.isSwerve());
    assertThrows(IllegalStateException.class, t::getDifferentialTrajectory);
  }

  @Test
  void autoCachesBothSampleTypes() {
    class TestAuto extends Auto {
      TestAuto() {
        loadTrajectory("diff");
        loadTrajectory("swerve");
      }
    }
    TestAuto auto = new TestAuto();
    auto.cacheTrajetories(false, n -> n.equals("diff") ? diffTraj() : swerveTraj());
    assertEquals(6, auto.getPath().length);
    assertEquals(0.0, auto.getStartPose().getX(), 1e-9);
    assertTrue(auto.getTrajectory("diff").get().isDifferential());
    assertTrue(auto.getTrajectory("swerve").get().isSwerve());
  }

  @Test
  void emptyAutoStartPoseIsZero() {
    assertEquals(Pose2d.kZero, new Auto().getStartPose());
  }
}
