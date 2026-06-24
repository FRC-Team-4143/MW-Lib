package com.marswars.proxy_server;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TagSolutionsInputs {
    /** Vision timestamp in seconds (sec + nsec/1e9) per solution */
    public double[] timestamps = new double[0];
    /** Robot pose estimate per solution */
    public Pose2d[] poses = new Pose2d[0];
    /** Camera serial per solution */
    public String[] cameraSerials = new String[0];
    /** Number of detected tag IDs per solution */
    public int[] tagIdCounts = new int[0];
    /** All detected tag IDs, concatenated across solutions */
    public int[] tagIds = new int[0];
}
