package com.marswars.regions;

/** Where the robot code's field coordinate system puts (0, 0). */
public enum Origin {
    /** The blue alliance wall's right corner, +x toward red. WPILib's default. */
    BLUE_CORNER,
    /** The center of the field, +x toward blue: the blue-corner system turned 180 degrees. */
    CENTER
}
