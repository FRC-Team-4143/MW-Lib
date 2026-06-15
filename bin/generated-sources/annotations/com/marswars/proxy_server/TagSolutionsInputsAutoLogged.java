package com.marswars.proxy_server;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class TagSolutionsInputsAutoLogged extends TagSolutionsInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Timestamps", timestamps);
    table.put("Poses", poses);
    table.put("CameraSerials", cameraSerials);
    table.put("TagIdCounts", tagIdCounts);
    table.put("TagIds", tagIds);
  }

  @Override
  public void fromLog(LogTable table) {
    timestamps = table.get("Timestamps", timestamps);
    poses = table.get("Poses", poses);
    cameraSerials = table.get("CameraSerials", cameraSerials);
    tagIdCounts = table.get("TagIdCounts", tagIdCounts);
    tagIds = table.get("TagIds", tagIds);
  }

  public TagSolutionsInputsAutoLogged clone() {
    TagSolutionsInputsAutoLogged copy = new TagSolutionsInputsAutoLogged();
    copy.timestamps = this.timestamps.clone();
    copy.poses = this.poses.clone();
    copy.cameraSerials = this.cameraSerials.clone();
    copy.tagIdCounts = this.tagIdCounts.clone();
    copy.tagIds = this.tagIds.clone();
    return copy;
  }
}
