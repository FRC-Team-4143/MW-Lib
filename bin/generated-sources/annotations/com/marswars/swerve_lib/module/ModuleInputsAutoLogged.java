package com.marswars.swerve_lib.module;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleInputsAutoLogged extends ModuleInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("DrivePositionRad", drivePositionRad);
    table.put("DriveVelocityRadPerSec", driveVelocityRadPerSec);
    table.put("DriveAppliedVolts", driveAppliedVolts);
    table.put("DriveCurrentAmps", driveCurrentAmps);
    table.put("SteerAbsolutePosition", steerAbsolutePosition);
    table.put("SteerVelocityRadPerSec", steerVelocityRadPerSec);
    table.put("SteerAppliedVolts", steerAppliedVolts);
    table.put("SteerCurrentAmps", steerCurrentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    drivePositionRad = table.get("DrivePositionRad", drivePositionRad);
    driveVelocityRadPerSec = table.get("DriveVelocityRadPerSec", driveVelocityRadPerSec);
    driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
    driveCurrentAmps = table.get("DriveCurrentAmps", driveCurrentAmps);
    steerAbsolutePosition = table.get("SteerAbsolutePosition", steerAbsolutePosition);
    steerVelocityRadPerSec = table.get("SteerVelocityRadPerSec", steerVelocityRadPerSec);
    steerAppliedVolts = table.get("SteerAppliedVolts", steerAppliedVolts);
    steerCurrentAmps = table.get("SteerCurrentAmps", steerCurrentAmps);
  }

  public ModuleInputsAutoLogged clone() {
    ModuleInputsAutoLogged copy = new ModuleInputsAutoLogged();
    copy.drivePositionRad = this.drivePositionRad;
    copy.driveVelocityRadPerSec = this.driveVelocityRadPerSec;
    copy.driveAppliedVolts = this.driveAppliedVolts;
    copy.driveCurrentAmps = this.driveCurrentAmps;
    copy.steerAbsolutePosition = this.steerAbsolutePosition;
    copy.steerVelocityRadPerSec = this.steerVelocityRadPerSec;
    copy.steerAppliedVolts = this.steerAppliedVolts;
    copy.steerCurrentAmps = this.steerCurrentAmps;
    return copy;
  }
}
