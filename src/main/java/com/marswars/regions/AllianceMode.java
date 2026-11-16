package com.marswars.regions;

/** Whether a region moves with the robot's alliance. */
public enum AllianceMode {
    /** Drawn as the blue alliance sees it, and mirrored to the other side of the field on red. */
    FLIPS,
    /** Fixed on the field, whatever the alliance. */
    FIXED
}
