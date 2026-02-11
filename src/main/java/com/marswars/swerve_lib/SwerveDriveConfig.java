package com.marswars.swerve_lib;

import com.marswars.swerve_lib.module.SwerveModuleConfig;

/**
 * Immutable configuration container for a full swerve drivetrain.
 */
public class SwerveDriveConfig {
    public final SwerveModuleConfig FL_MODULE_CONSTANTS;
    public final SwerveModuleConfig FR_MODULE_CONSTANTS;
    public final SwerveModuleConfig BL_MODULE_CONSTANTS;
    public final SwerveModuleConfig BR_MODULE_CONSTANTS;

    public final int PIGEON2_ID;
    public final String PIGEON2_CANBUS_NAME;

    /**
     * Creates a new swerve drive configuration.
     *
     * @param fl_module_constants Front-left module configuration
     * @param fr_module_constants Front-right module configuration
     * @param bl_module_constants Back-left module configuration
     * @param br_module_constants Back-right module configuration
     * @param pigeon2_id Pigeon2 CAN device ID
     * @param pigeon2_canbus_name Pigeon2 CAN bus name
     */
    public SwerveDriveConfig(
            SwerveModuleConfig fl_module_constants,
            SwerveModuleConfig fr_module_constants,
            SwerveModuleConfig bl_module_constants,
            SwerveModuleConfig br_module_constants,
            int pigeon2_id,
            String pigeon2_canbus_name) {
        FL_MODULE_CONSTANTS = fl_module_constants;
        FR_MODULE_CONSTANTS = fr_module_constants;
        BL_MODULE_CONSTANTS = bl_module_constants;
        BR_MODULE_CONSTANTS = br_module_constants;

        PIGEON2_ID = pigeon2_id;
        PIGEON2_CANBUS_NAME = pigeon2_canbus_name;
    }
}
