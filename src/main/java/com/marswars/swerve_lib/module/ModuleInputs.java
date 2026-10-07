package com.marswars.swerve_lib.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ModuleInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveStatorCurrentAmps = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public Rotation2d steerAbsolutePosition = new Rotation2d();
    public double steerVelocityRadPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerStatorCurrentAmps = 0.0;
    public double steerSupplyCurrentAmps = 0.0;
    public double encoderAbsolutePosition = 0.0;
}
