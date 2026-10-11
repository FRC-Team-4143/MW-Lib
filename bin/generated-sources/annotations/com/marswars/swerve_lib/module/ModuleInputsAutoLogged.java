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
    table.put("DriveStatorCurrentAmps", driveStatorCurrentAmps);
    table.put("DriveSupplyCurrentAmps", driveSupplyCurrentAmps);
    table.put("SteerAbsolutePosition", steerAbsolutePosition);
    table.put("SteerVelocityRadPerSec", steerVelocityRadPerSec);
    table.put("SteerAppliedVolts", steerAppliedVolts);
    table.put("SteerStatorCurrentAmps", steerStatorCurrentAmps);
    table.put("SteerSupplyCurrentAmps", steerSupplyCurrentAmps);
    table.put("EncoderAbsolutePosition", encoderAbsolutePosition);
  }

  @Override
  public void fromLog(LogTable table) {
    drivePositionRad = table.get("DrivePositionRad", drivePositionRad);
    driveVelocityRadPerSec = table.get("DriveVelocityRadPerSec", driveVelocityRadPerSec);
    driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
    driveStatorCurrentAmps = table.get("DriveStatorCurrentAmps", driveStatorCurrentAmps);
    driveSupplyCurrentAmps = table.get("DriveSupplyCurrentAmps", driveSupplyCurrentAmps);
    steerAbsolutePosition = table.get("SteerAbsolutePosition", steerAbsolutePosition);
    steerVelocityRadPerSec = table.get("SteerVelocityRadPerSec", steerVelocityRadPerSec);
    steerAppliedVolts = table.get("SteerAppliedVolts", steerAppliedVolts);
    steerStatorCurrentAmps = table.get("SteerStatorCurrentAmps", steerStatorCurrentAmps);
    steerSupplyCurrentAmps = table.get("SteerSupplyCurrentAmps", steerSupplyCurrentAmps);
    encoderAbsolutePosition = table.get("EncoderAbsolutePosition", encoderAbsolutePosition);
  }

  public ModuleInputsAutoLogged clone() {
    ModuleInputsAutoLogged copy = new ModuleInputsAutoLogged();
    copy.drivePositionRad = this.drivePositionRad;
    copy.driveVelocityRadPerSec = this.driveVelocityRadPerSec;
    copy.driveAppliedVolts = this.driveAppliedVolts;
    copy.driveStatorCurrentAmps = this.driveStatorCurrentAmps;
    copy.driveSupplyCurrentAmps = this.driveSupplyCurrentAmps;
    copy.steerAbsolutePosition = this.steerAbsolutePosition;
    copy.steerVelocityRadPerSec = this.steerVelocityRadPerSec;
    copy.steerAppliedVolts = this.steerAppliedVolts;
    copy.steerStatorCurrentAmps = this.steerStatorCurrentAmps;
    copy.steerSupplyCurrentAmps = this.steerSupplyCurrentAmps;
    copy.encoderAbsolutePosition = this.encoderAbsolutePosition;
    return copy;
  }
}
