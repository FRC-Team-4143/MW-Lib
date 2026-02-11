package com.marswars.swerve_lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Data containers for time-stamped swerve odometry measurements.
 */
public class SwerveMeasurements {
    /** Time-stamped module position measurement. */
    public static class ModuleMeasurement {
        public double timestamp;
        public SwerveModulePosition module_positions;
    }

    /** Time-stamped gyro measurement. */
    public static class GyroMeasurement {
        public double timestamp;
        public Rotation2d gyro_yaw;
    }

    /** Aggregated time-stamped swerve measurement bundle. */
    public static class SwerveMeasurement {
        public double timestamp;
        public SwerveModulePosition[] module_positions;
        public Rotation2d gyro_yaw;
    }
}
