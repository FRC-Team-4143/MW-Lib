package com.marswars.util;
import org.wpilib.driverstation.DriverStationErrors;

import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.system.Filesystem;
import java.io.IOException;

public abstract class TagLayouts {
    public static AprilTagFieldLayout getTagLayoutFromPath(String path) {
        AprilTagFieldLayout layout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        try {
            layout =
                    new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve(path));
        } catch (IOException E) {
            DriverStationErrors.reportWarning(
                    "Unable to find path to aprilTagFeild, k2025ReefscapeWelded used", false);
        }
        return layout;
    }
}
