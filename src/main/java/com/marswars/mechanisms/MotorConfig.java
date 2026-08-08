package com.marswars.mechanisms;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;

public class MotorConfig {

    public enum TalonMotorType {
        X60,
        X44,
        FALCON500,
        MINION,
        NEO_550,
    }

    public String canbus_name = "rio";

    public int can_id = 1;

    public TalonMotorType motor_type = TalonMotorType.X60;

    private Object config;

    public MotorConfig(MotorConfig config) {
        this.canbus_name = config.canbus_name;
        this.can_id = config.can_id;
        this.motor_type = config.motor_type;
        this.config = config.config;
    }

    public MotorConfig() {}

    /**
     * Returns true if this motor uses TalonFXS configuration (MINION, NEO_550)
     */
    public boolean isFXS() {
        return motor_type == TalonMotorType.MINION || motor_type == TalonMotorType.NEO_550;
    }

    /**
     * Applies a TalonFXConfiguration to this motor config. Throws exception if motor uses FXS config.
     * @param fxConfig The TalonFXConfiguration to apply
     */
    public void apply(TalonFXConfiguration fxConfig) {
        if (isFXS()) {
            throw new IllegalStateException("Motor type " + motor_type + " uses TalonFXS configuration");
        }
        config = fxConfig;
    }

    /**
     * Applies a TalonFXSConfiguration to this motor config. Throws exception if motor uses FX config.
     * @param fxsConfig The TalonFXSConfiguration to apply
     */
    public void apply(TalonFXSConfiguration fxsConfig) {
        if (!isFXS()) {
            throw new IllegalStateException("Motor type " + motor_type + " uses TalonFX configuration");
        }
        config = fxsConfig;
    }

    /**
     * Gets the config as TalonFXConfiguration. Throws exception if motor uses FXS config.
     * If config is null, creates a default TalonFXConfiguration.
     */
    public TalonFXConfiguration getAsFXConfig() {
        if (isFXS()) {
            throw new IllegalStateException("Motor type " + motor_type + " uses TalonFXS configuration");
        }
        if (config == null) {
            config = new TalonFXConfiguration();
        }
        return (TalonFXConfiguration) config;
    }

    /**
     * Gets the config as TalonFXSConfiguration. Throws exception if motor uses FX config.
     * If config is null, creates a default TalonFXSConfiguration.
     */
    public TalonFXSConfiguration getAsFXSConfig() {
        if (!isFXS()) {
            throw new IllegalStateException("Motor type " + motor_type + " uses TalonFX configuration");
        }
        if (config == null) {
            config = new TalonFXSConfiguration();
        }
        return (TalonFXSConfiguration) config;
    }
}
