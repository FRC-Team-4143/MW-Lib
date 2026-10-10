package com.marswars.swerve_lib;

import edu.wpi.first.math.geometry.Rotation2d;

/** The direction the driver faces, used as "forward" for operator-perspective driving. */
public enum OperatorPerspective {
    BLUE_ALLIANCE(Rotation2d.fromDegrees(0.0)),
    RED_ALLIANCE(Rotation2d.fromDegrees(180.0));

    private OperatorPerspective(Rotation2d heading) {
        this.heading = heading;
    }

    public final Rotation2d heading;
}
