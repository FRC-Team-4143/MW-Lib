package com.marswars.swerve_lib;

/** Swerve drive operational states defining different control modes. */
public enum SwerveStates {
    /** Drive relative to the robot's orientation using joystick inputs. */
    ROBOT_CENTRIC,
    /** Drive relative to the field's orientation using joystick inputs. */
    FIELD_CENTRIC,
    /** Follow a pre-planned Choreo trajectory path with Choreo rotation control. */
    CHOREO_PATH,
    /** Drive towards a target pose using feedback control. */
    TRACTOR_BEAM,
    /** Drive using raw ChassisSpeeds including rotation. */
    CHASSIS_SPEEDS,
    /** Slow precision movement relative to robot orientation using POV/D-pad. */
    CRAWL_ROBOT_CENTRIC,
    /** Slow precision movement relative to field orientation using POV/D-pad. */
    CRAWL_FIELD_CENTRIC,
    /** Drive relative to robot orientation with rotation locked to a target heading. */
    ROBOT_CENTRIC_ROTATION_LOCK,
    /** Drive relative to field orientation with rotation locked to a target heading. */
    FIELD_CENTRIC_ROTATION_LOCK,
    /** Follow a Choreo trajectory path with rotation locked to a target heading. */
    CHOREO_PATH_ROTATION_LOCK,
    /** Drive using raw ChassisSpeeds with rotation locked to a target heading. */
    CHASSIS_SPEEDS_ROTATION_LOCK,
    /** Slow precision movement relative to robot with rotation locked to a target heading. */
    CRAWL_ROBOT_CENTRIC_ROTATION_LOCK,
    /** Slow precision movement relative to field with rotation locked to a target heading. */
    CRAWL_FIELD_CENTRIC_ROTATION_LOCK,
    /** Brake mode locking the wheels in an x pattern */
    BRAKE,
    /** Manual tuning mode for testing chassis speeds. */
    TUNING,
    /** Idle state with no movement commands. */
    IDLE
}
