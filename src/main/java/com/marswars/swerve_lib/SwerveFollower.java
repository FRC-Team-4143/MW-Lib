package com.marswars.swerve_lib;

import com.marswars.auto.ChoreoTrajectory;

/**
 * What the behavior-tree swerve nodes ({@code com.marswars.bt.swerve.SwerveNodes}) need from a
 * drivetrain: follow a Choreo path, report how it is going, and take a swerve state. {@link
 * MwSwerveSubsystem} implements it; tests use a small fake.
 */
public interface SwerveFollower {
    /** Selects the Choreo path to follow (already alliance-flipped). */
    void setDesiredChoreoTrajectory(ChoreoTrajectory trajectory);

    /** Requests a swerve state. */
    void setWantedState(SwerveStates state);

    /** True at the end pose, at the end velocity, with the whole path time elapsed. */
    boolean isAtChoreoSetpoint();

    /** True once at least {@code seconds} of the path timer have run. */
    boolean hasChoreoTimeElapsed(double seconds);

    /** True once the running path has passed the named event marker. */
    boolean hasChoreoEventBeenPassed(String event_name);

    /** True when the chassis is not moving. */
    boolean isChassisStationary();

    /**
     * True while following a path with its timer paused, which happens while the robot lags the
     * path (pushed, blocked, bumped). False outside a Choreo state.
     */
    boolean isChoreoTimerPaused();
}
