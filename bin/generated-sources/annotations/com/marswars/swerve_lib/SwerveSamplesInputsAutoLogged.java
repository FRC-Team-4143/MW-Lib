package com.marswars.swerve_lib;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveSamplesInputsAutoLogged extends SwerveSamplesInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Timestamps", timestamps);
    table.put("GyroYawsRad", gyroYawsRad);
    table.put("ModuleDistancesMeters", moduleDistancesMeters);
    table.put("ModuleSteerAnglesRad", moduleSteerAnglesRad);
  }

  @Override
  public void fromLog(LogTable table) {
    timestamps = table.get("Timestamps", timestamps);
    gyroYawsRad = table.get("GyroYawsRad", gyroYawsRad);
    moduleDistancesMeters = table.get("ModuleDistancesMeters", moduleDistancesMeters);
    moduleSteerAnglesRad = table.get("ModuleSteerAnglesRad", moduleSteerAnglesRad);
  }

  public SwerveSamplesInputsAutoLogged clone() {
    SwerveSamplesInputsAutoLogged copy = new SwerveSamplesInputsAutoLogged();
    copy.timestamps = this.timestamps.clone();
    copy.gyroYawsRad = this.gyroYawsRad.clone();
    copy.moduleDistancesMeters = this.moduleDistancesMeters.clone();
    copy.moduleSteerAnglesRad = this.moduleSteerAnglesRad.clone();
    return copy;
  }
}
