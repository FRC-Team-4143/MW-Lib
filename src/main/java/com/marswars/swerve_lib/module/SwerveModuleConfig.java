package com.marswars.swerve_lib.module;

import com.marswars.util.FxMotorConfig;

/**
 * Configuration container for a single swerve module.
 */
public class SwerveModuleConfig {
    /** Encoder hardware options for the module. */
    public enum EncoderType {
        CTRE_CAN_CODER,
        ANALOG_ENCODER
    }

    // Module hardware configuration fields
    public EncoderType encoder_type;
    public int encoder_id;
    public FxMotorConfig drive_motor_config;
    public FxMotorConfig steer_motor_config;

    // Module type
    public ModuleType module_type;

    // module geometry
    public double wheel_radius_m;
    public double encoder_offset_rad;
    public double speed_at_12_volts;
    public double location_x;
    public double location_y;

    public boolean enable_foc = false;
}
