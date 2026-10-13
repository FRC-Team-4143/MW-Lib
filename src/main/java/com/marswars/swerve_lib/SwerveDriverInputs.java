package com.marswars.swerve_lib;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Raw driver joystick inputs consumed by {@link MwSwerveSubsystem} for teleop and crawl driving.
 * Axes are the raw [-1, 1] controller values (WPILib convention: stick forward is negative Y);
 * the subsystem applies deadband, negation and scaling.
 *
 * @param left_x left stick X axis (strafe)
 * @param left_y left stick Y axis (forward/back)
 * @param right_x right stick X axis (rotation)
 * @param pov POV/D-pad direction, empty when not pressed (drives the crawl states)
 */
public record SwerveDriverInputs(
        DoubleSupplier left_x,
        DoubleSupplier left_y,
        DoubleSupplier right_x,
        Supplier<Optional<Rotation2d>> pov) {

    /** Inputs that always read zero / no POV; useful for tests and robots without a driver. */
    public static SwerveDriverInputs none() {
        return new SwerveDriverInputs(() -> 0.0, () -> 0.0, () -> 0.0, Optional::empty);
    }
}
