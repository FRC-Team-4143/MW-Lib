package com.marswars.mechanisms;

import org.littletonrobotics.junction.AutoLog;

/**
 * AdvantageKit inputs struct for all TalonFX-based mechanisms (RollerMech, FlywheelMech, ArmMech,
 * ElevatorMech). Fields hold only the sensor reads that must be captured in the log for
 * deterministic replay — command targets are outputs and stay in the mechanism class.
 */
@AutoLog
public class MechInputs {
    public double position = 0.0;
    public double velocity = 0.0;
    public double[] appliedVoltage      = new double[0];
    public double[] supplyCurrentDraw   = new double[0];
    public double[] statorCurrentDraw   = new double[0];
    public double[] torqueCurrentDraw   = new double[0];
    public double[] motorTempC          = new double[0];
    public double[] busVoltage          = new double[0];
}
