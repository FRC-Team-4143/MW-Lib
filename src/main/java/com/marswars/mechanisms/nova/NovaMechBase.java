package com.marswars.mechanisms.nova;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig.PIDConfiguration;
import com.marswars.mechanisms.MechBase;
import com.marswars.util.NovaMotorConfig;
import com.marswars.util.NovaMotorConfig.NovaMotorType;

import java.util.List;
import java.util.function.Function;

/**
 * ThriftyNova-specific mechanism base class.
 * Provides motor configuration helpers for Thrifty Nova motor controllers.
 */
public abstract class NovaMechBase extends MechBase<ThriftyNova, NovaMotorConfig> {

    /**
     * Creates a new ThriftyNova-based mechanism
     * @param logging_prefix the logging prefix for this mechanism
     */
    public NovaMechBase(String logging_prefix) {
        super(logging_prefix);
    }

    /**
     * Creates a new ThriftyNova-based mechanism with a specified mechanism name
     * @param logging_prefix the logging prefix for this mechanism
     * @param mech_name the name of the mechanism (used for multiple instances of the same mech)
     */
    public NovaMechBase(String logging_prefix, String mech_name) {
        super(logging_prefix, mech_name);
    }

    /**
     * Configures the ThriftyNova motors based on the given motor configs
     *
     * @param motor_configs the list of motor configs
     * @param sensor_to_mech_ratio the sensor to mechanism ratio (gear ratio)
     * @param configMaster a function to modify the master motor config before applying it
     * @return the constructed motors
     */
    @Override
    public ConstructedMotors configMotors(
            List<NovaMotorConfig> motor_configs,
            double sensor_to_mech_ratio,
            Function<NovaMotorConfig, NovaMotorConfig> configMaster) {
        // throw a fit if we don't have any motors
        if (motor_configs == null || motor_configs.size() == 0) {
            throw new IllegalArgumentException("Motor configs is null or empty");
        }
        ConstructedMotors constructed = new ConstructedMotors();
        constructed.motors = new ThriftyNova[motor_configs.size()];
        
        for (int i = 0; i < motor_configs.size(); i++) {
            NovaMotorConfig cfg = motor_configs.get(i);
            if (cfg.canbus_name == null || cfg.canbus_name.isEmpty()) {
                throw new IllegalArgumentException("Motor canbus name is null or empty");
            }

            // Create motor with appropriate motor type
            // Map our NovaMotorType to ThriftyNova.MotorType
            // MINION motors need the MINION type, all REV motors (VORTEX, NEO_550, PULSAR_775) use default (NEO)
            if (cfg.motor_type == NovaMotorType.MINION) {
                constructed.motors[i] = new ThriftyNova(cfg.can_id, ThriftyNova.MotorType.MINION);
            } else {
                // VORTEX, NEO_550, PULSAR_775 all use default NEO motor type
                constructed.motors[i] = new ThriftyNova(cfg.can_id, ThriftyNova.MotorType.NEO);
            }

            constructed.motors[i].setNTLogging(false);

            if (configMaster != null) {
                cfg = configMaster.apply(cfg);
            }

            // Ensure PID configs are initialized before applying (defensive coding)
            ensurePIDConfigInitialized(cfg.config.pid0);
            ensurePIDConfigInitialized(cfg.config.pid1);

            // Note: Unlike TalonFX, ThriftyNova does not have built-in support for conversion factors.
            // The sensor_to_mech_ratio parameter is provided for API consistency but may need to be applied differently based on ThriftyLib API.
            
            // Apply the configs to the motor
            constructed.motors[i].applyConfig(cfg.config);

            // Make follower motors (non-master motors)
            if (i > 0) {
                // ThriftyNova follow() takes the CAN ID of the motor to follow
                constructed.motors[i].follow(motor_configs.get(0).can_id);
            }
        }

        return constructed;
    }

    /**
     * Ensures PID configuration fields are initialized to prevent NullPointerException.
     * This is needed when motors aren't connected or configs aren't loaded from file.
     * 
     * @param config The PID configuration to initialize
     */
    protected static void ensurePIDConfigInitialized(PIDConfiguration config) {
        if (config.p == null) config.p = 0.0;
        if (config.i == null) config.i = 0.0;
        if (config.d == null) config.d = 0.0;
        if (config.f == null) config.f = 0.0;
    }
}
