package com.marswars.regions;

/** Whether a region moves with the robot's alliance. */
public enum AllianceMode {
    /** Drawn from blue's side, and moved to red's side when the robot is on red. */
    FLIPS,
    /** Fixed on the field, whatever the alliance. */
    FIXED
}
