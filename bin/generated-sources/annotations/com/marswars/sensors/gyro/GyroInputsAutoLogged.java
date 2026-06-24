package com.marswars.sensors.gyro;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroInputsAutoLogged extends GyroInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("ConnectedDebounced", connectedDebounced);
    table.put("YawPosition", yawPosition);
    table.put("PitchPosition", pitchPosition);
    table.put("RollPosition", rollPosition);
    table.put("YawVelocityRadPerSec", yawVelocityRadPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    connectedDebounced = table.get("ConnectedDebounced", connectedDebounced);
    yawPosition = table.get("YawPosition", yawPosition);
    pitchPosition = table.get("PitchPosition", pitchPosition);
    rollPosition = table.get("RollPosition", rollPosition);
    yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", yawVelocityRadPerSec);
  }

  public GyroInputsAutoLogged clone() {
    GyroInputsAutoLogged copy = new GyroInputsAutoLogged();
    copy.connected = this.connected;
    copy.connectedDebounced = this.connectedDebounced;
    copy.yawPosition = this.yawPosition;
    copy.pitchPosition = this.pitchPosition;
    copy.rollPosition = this.rollPosition;
    copy.yawVelocityRadPerSec = this.yawVelocityRadPerSec;
    return copy;
  }
}
