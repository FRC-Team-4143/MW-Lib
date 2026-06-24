package com.marswars.sensors.tof;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class TofInputsAutoLogged extends TofInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Range", range);
  }

  @Override
  public void fromLog(LogTable table) {
    range = table.get("Range", range);
  }

  public TofInputsAutoLogged clone() {
    TofInputsAutoLogged copy = new TofInputsAutoLogged();
    copy.range = this.range;
    return copy;
  }
}
