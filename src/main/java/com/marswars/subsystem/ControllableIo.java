package com.marswars.subsystem;

import java.util.List;

/**
 * An IO the {@link SubsystemManager} can switch off, override sensor readings on and, for
 * mechanisms, drive by hand. Implemented by the library's IO classes (every {@code *Mech}, {@code
 * Tof}, {@code Gyro}); the manager attaches an {@link IoControls} to each one it finds in a
 * registered subsystem's {@link MwSubsystemBase#getIos()}, and to their {@link #getChildIos()}.
 */
public interface ControllableIo extends SubsystemIoBase {

    /**
     * @return the name used for this IO's entries at {@code /MwLib/Subsystems/<Sub>/IO/<Name>/}
     */
    String getIoName();

    /**
     * @return the prefix of this IO's tunable keys (its logging key), so its tunables only change
     *     while its subsystem is in {@link SubsystemControl#MANUAL}
     */
    String getLoggingKey();

    /**
     * Called once by the manager. Declare the sensor channels the IO can override with {@link
     * IoControls#declareSensor(String)} and keep the reference for getters and {@link
     * #isNeutralOutputs()}.
     *
     * @param controls this IO's controls
     */
    void attachControls(IoControls controls);

    /** @return IOs this one owns and updates itself (e.g. the gyro inside a swerve drive) */
    default List<ControllableIo> getChildIos() {
        return List.of();
    }

    /** @return the manual modes this IO supports; empty means it has no outputs to drive */
    default List<ManualMode> getManualModes() {
        return List.of();
    }

    /**
     * Drives the outputs by hand. Called every loop while the subsystem is {@link
     * SubsystemControl#MANUAL}, with a mode from {@link #getManualModes()} other than {@link
     * ManualMode#NEUTRAL} (neutral is applied through {@link IoControls#holdsNeutral()}).
     *
     * @param mode the output mode
     * @param setpoint the setpoint in the units of the matching {@code setTarget*} method
     */
    default void applyManualOutput(ManualMode mode, double setpoint) {}

    /**
     * @param mode the mode manual control starts in
     * @return the setpoint manual control starts at, so nothing moves on entry (the measured
     *     position for position modes, zero otherwise)
     */
    default double getManualSeed(ManualMode mode) {
        return 0.0;
    }

    /**
     * @return true while outputs must be neutral: the subsystem is OFF, this IO is switched off, or
     *     it is in manual {@link ManualMode#NEUTRAL}
     */
    default boolean isNeutralOutputs() {
        return false;
    }
}
