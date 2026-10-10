package com.marswars.vision;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import com.marswars.proxy_server.TagSolutionPacket.TagSolutionData;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFields;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Runs the real PhotonLib vision simulation (OpenCV / AprilTag JNI) headlessly with the HAL in
 * simulation mode and real-time sleeps. Skipped when the vision natives were not extracted (an
 * offline build without them); see {@code visionSimNativeLibs} in build.gradle.
 */
class MwVisionSimTest {
  private static MwVisionSim sim_;

  @BeforeAll
  static void setUp() {
    String lib_dir = System.getProperty("java.library.path");
    assumeTrue(
        Files.exists(Path.of(lib_dir, "libopencv_java4100.so")),
        "vision natives not extracted, skipping");
    assertTrue(HAL.initialize(500, 0));
    sim_ = new MwVisionSim(AprilTagFields.k2026RebuiltWelded);
    sim_.addCamera(
        "test-front",
        new Transform3d(new Translation3d(0.3, 0, 0.5), new Rotation3d(0, Math.toRadians(-20), 0)));
  }

  /** Updates at 50 Hz in real time and returns every solution handed out. */
  private static List<TagSolutionData> run(Pose2d pose, double seconds) throws InterruptedException {
    List<TagSolutionData> out = new ArrayList<>();
    for (int i = 0; i < seconds * 50; i++) {
      sim_.update(pose);
      out.addAll(sim_.getTagSolutions(pose));
      Thread.sleep(20);
    }
    return out;
  }

  /** Lets the sim settle on a new pose (latency, stale pictures), then collects solutions. */
  private static List<TagSolutionData> settleAndRun(Pose2d pose) throws InterruptedException {
    run(pose, 0.3);
    return run(pose, 0.5);
  }

  private static double secondsOf(TagSolutionData s) {
    return s.timestamp.getSeconds();
  }

  @Test
  void multiTagSolutionMatchesTruePose() throws Exception {
    Pose2d truth = new Pose2d(1.0, 1.0, Rotation2d.kZero); // 2 tags visible here
    var solutions = settleAndRun(truth);

    assertFalse(solutions.isEmpty());
    for (var s : solutions) {
      assertTrue(s.detectedIds.size() >= 2, "multi-tag solve must report >= 2 used ids");
      assertEquals(0.0, s.pose.getTranslation().getDistance(truth.getTranslation()), 0.15);
      assertEquals(0.0, s.pose.getRotation().minus(truth.getRotation()).getDegrees(), 3.0);
    }
  }

  @Test
  void singleTagSolutionReportsOneId() throws Exception {
    Pose2d truth = new Pose2d(2.0, 1.0, Rotation2d.kZero); // exactly 1 tag visible here
    var solutions = settleAndRun(truth);

    assertFalse(solutions.isEmpty());
    for (var s : solutions) {
      assertEquals(1, s.detectedIds.size());
    }
  }

  @Test
  void eachPictureIsEmittedOnce() throws Exception {
    // 50 Hz calls against a 30 fps camera: the same picture must not be handed out twice
    var solutions = settleAndRun(new Pose2d(1.0, 1.0, Rotation2d.kZero));

    assertFalse(solutions.isEmpty());
    var seen = new java.util.HashSet<Double>();
    for (var s : solutions) {
      assertTrue(seen.add(secondsOf(s)), "duplicate timestamp " + secondsOf(s));
    }
    // ~0.5 s at 30 fps is ~15 pictures; far fewer than the 25 calls' worth of duplicates
    assertTrue(solutions.size() <= 20, "got " + solutions.size());
  }
}
