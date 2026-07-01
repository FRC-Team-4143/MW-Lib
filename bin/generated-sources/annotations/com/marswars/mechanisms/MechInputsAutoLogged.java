package com.marswars.mechanisms;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class MechInputsAutoLogged extends MechInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Position", position);
    table.put("Velocity", velocity);
    table.put("AppliedVoltage", appliedVoltage);
    table.put("SupplyCurrentDraw", supplyCurrentDraw);
    table.put("StatorcurrentDraw", statorcurrentDraw);
    table.put("MotorTempC", motorTempC);
    table.put("BusVoltage", busVoltage);
  }

  @Override
  public void fromLog(LogTable table) {
    position = table.get("Position", position);
    velocity = table.get("Velocity", velocity);
    appliedVoltage = table.get("AppliedVoltage", appliedVoltage);
    supplyCurrentDraw = table.get("SupplyCurrentDraw", supplyCurrentDraw);
    statorcurrentDraw = table.get("StatorcurrentDraw", statorcurrentDraw);
    motorTempC = table.get("MotorTempC", motorTempC);
    busVoltage = table.get("BusVoltage", busVoltage);
  }

  public MechInputsAutoLogged clone() {
    MechInputsAutoLogged copy = new MechInputsAutoLogged();
    copy.position = this.position;
    copy.velocity = this.velocity;
    copy.appliedVoltage = this.appliedVoltage.clone();
    copy.supplyCurrentDraw = this.supplyCurrentDraw.clone();
    copy.statorcurrentDraw = this.statorcurrentDraw.clone();
    copy.motorTempC = this.motorTempC.clone();
    copy.busVoltage = this.busVoltage.clone();
    return copy;
  }
}
