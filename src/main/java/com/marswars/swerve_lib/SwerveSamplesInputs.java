package com.marswars.swerve_lib;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SwerveSamplesInputs {
    /** Timestamps for each swerve measurement sample (seconds) */
    public double[] timestamps = new double[0];
    /** Gyro yaw for each sample (radians) */
    public double[] gyroYawsRad = new double[0];
    /** Module distances, flattened: index = sampleIndex * 4 + moduleIndex (meters) */
    public double[] moduleDistancesMeters = new double[0];
    /** Module steer angles, flattened: index = sampleIndex * 4 + moduleIndex (radians) */
    public double[] moduleSteerAnglesRad = new double[0];
}
