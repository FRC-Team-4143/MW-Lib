package com.marswars.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class GyroInputs {
    public boolean connected = false;
    public boolean connectedDebounced = false;
    public Rotation2d yawPosition = new Rotation2d();
    public Rotation2d pitchPosition = new Rotation2d();
    public Rotation2d rollPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;
}
