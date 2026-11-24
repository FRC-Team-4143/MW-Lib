package com.marswars.subsystem;

/**
 * An IO that can be switched off on its own, and held still when its subsystem enters {@link
 * SubsystemMode#TUNING}. Every {@code *Mech} implements it; {@link SubsystemManager} gives each one
 * an {@code /MwLib/Subsystems/<Sub>/Mechs/<Name>/Enabled} switch.
 */
public interface SwitchableIo extends SubsystemIoBase {
    /** @return the name used for this IO's switch */
    String getIoName();

    /** @return the IO's logging key (its tunables live under {@code /Tuning/<key>}) */
    String getLoggingKey();

    /** Called once on entering TUNING: hold where it is (or stop, for velocity mechanisms). */
    void hold();
}
