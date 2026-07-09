package com.marswars.mechanisms;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.marswars.util.ConstantsLoader;

public class MotorConfig {

    protected final ConstantsLoader loader = ConstantsLoader.getInstance();

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
        if (config.config instanceof TalonFXConfiguration) {
            this.config = ((TalonFXConfiguration) config.config).clone();
        } else if (config.config instanceof TalonFXSConfiguration) {
            this.config = ((TalonFXSConfiguration) config.config).clone();
        } else {
            this.config = config.config;
        }
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

    public void loadFromConfig(String... base_steps) {
        canbus_name = loader.getStringValue(ConstantsLoader.combinePath(base_steps, "bus_name"));
        can_id = loader.getIntValue(ConstantsLoader.combinePath(base_steps, "bus_id"));

        String motor_type_str =
                loader.getStringValue(ConstantsLoader.combinePath(base_steps, "type"));
        if (motor_type_str.equals("X60")) {
            motor_type = TalonMotorType.X60;
        } else if (motor_type_str.equals("X44")) {
            motor_type = TalonMotorType.X44;
        } else if (motor_type_str.equals("FALCON500")) {
            motor_type = TalonMotorType.FALCON500;
        } else if (motor_type_str.equals("MINION")) {
            motor_type = TalonMotorType.MINION;
        } else if (motor_type_str.equals("NEO_550")) {
            motor_type = TalonMotorType.NEO_550;
        } else {
            throw new RuntimeException("Unknown motor type: " + motor_type_str);
        }

        // Create the appropriate config type based on motor type
        if (isFXS()) {
            config = new TalonFXSConfiguration();
            loadFXSConfig(base_steps);
        } else {
            config = new TalonFXConfiguration();
            loadFXConfig(base_steps);
        }
    }

    private void loadFXConfig(String... base_steps) {
        TalonFXConfiguration fxConfig = (TalonFXConfiguration) config;

        // Load the base motor configs
        fxConfig.MotorOutput.Inverted =
                loader.getBoolValue(ConstantsLoader.combinePath(base_steps, "inverted"))
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        fxConfig.MotorOutput.NeutralMode =
                loader.getBoolValue(ConstantsLoader.combinePath(base_steps, "brake_mode"))
                        ? com.ctre.phoenix6.signals.NeutralModeValue.Brake
                        : com.ctre.phoenix6.signals.NeutralModeValue.Coast;

        // Load the slot configs
        fxConfig.Slot0.kS =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "ks"));
        fxConfig.Slot0.kV =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kv"));
        fxConfig.Slot0.kA =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "ka"));
        fxConfig.Slot0.kG =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kg"));
        fxConfig.Slot0.kP =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kp"));
        fxConfig.Slot0.kI =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "ki"));
        fxConfig.Slot0.kD =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kd"));
        fxConfig.Slot1.kS =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "ks"));
        fxConfig.Slot1.kV =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kv"));
        fxConfig.Slot1.kA =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "ka"));
        fxConfig.Slot1.kG =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kg"));
        fxConfig.Slot1.kP =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kp"));
        fxConfig.Slot1.kI =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "ki"));
        fxConfig.Slot1.kD =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kd"));

        // Load the motion magic configs
        if (loader.getBoolValue(
                ConstantsLoader.combinePath(base_steps, "motion_magic", "enabled"))) {
            fxConfig.MotionMagic.MotionMagicCruiseVelocity =
                    loader.getDoubleValue(
                            ConstantsLoader.combinePath(
                                    base_steps, "motion_magic", "cruise_velocity"));
            fxConfig.MotionMagic.MotionMagicAcceleration =
                    loader.getDoubleValue(
                            ConstantsLoader.combinePath(
                                    base_steps, "motion_magic", "acceleration"));
            fxConfig.MotionMagic.MotionMagicJerk =
                    loader.getDoubleValue(
                            ConstantsLoader.combinePath(base_steps, "motion_magic", "jerk"));
        }

        // Load a supply current limit if configured
        if (loader.getBoolValue(
                ConstantsLoader.combinePath(base_steps, "supply_limit", "enabled"))) {
            fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            fxConfig.CurrentLimits.SupplyCurrentLimit =
                    loader.getDoubleValue(
                            ConstantsLoader.combinePath(
                                    base_steps, "supply_limit", "current_limit"));
            fxConfig.CurrentLimits.SupplyCurrentLowerTime =
                    loader.getDoubleValue(
                            ConstantsLoader.combinePath(
                                    base_steps, "supply_limit", "trigger_time"));
        }

        // Load a stator current limit if configured
        if (loader.getBoolValue(
                ConstantsLoader.combinePath(base_steps, "stator_limit", "enabled"))) {
            fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            fxConfig.CurrentLimits.StatorCurrentLimit =
                    loader.getDoubleValue(
                            ConstantsLoader.combinePath(
                                    base_steps, "stator_limit", "current_limit"));
        }
    }

    private void loadFXSConfig(String... base_steps) {
        TalonFXSConfiguration fxsConfig = (TalonFXSConfiguration) config;

        // Load the base motor configs
        fxsConfig.MotorOutput.Inverted =
                loader.getBoolValue(ConstantsLoader.combinePath(base_steps, "inverted"))
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        fxsConfig.MotorOutput.NeutralMode =
                loader.getBoolValue(ConstantsLoader.combinePath(base_steps, "brake_mode"))
                        ? com.ctre.phoenix6.signals.NeutralModeValue.Brake
                        : com.ctre.phoenix6.signals.NeutralModeValue.Coast;

        // Load the slot configs
        fxsConfig.Slot0.kS =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "ks"));
        fxsConfig.Slot0.kV =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kv"));
        fxsConfig.Slot0.kA =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "ka"));
        fxsConfig.Slot0.kG =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kg"));
        fxsConfig.Slot0.kP =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kp"));
        fxsConfig.Slot0.kI =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "ki"));
        fxsConfig.Slot0.kD =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kd"));
        fxsConfig.Slot1.kS =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "ks"));
        fxsConfig.Slot1.kV =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kv"));
        fxsConfig.Slot1.kA =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "ka"));
        fxsConfig.Slot1.kG =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kg"));
        fxsConfig.Slot1.kP =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kp"));
        fxsConfig.Slot1.kI =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "ki"));
        fxsConfig.Slot1.kD =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kd"));

        // Load the motion magic configs
        if (loader.getBoolValue(
                ConstantsLoader.combinePath(base_steps, "motion_magic", "enabled"))) {
            fxsConfig.MotionMagic.MotionMagicCruiseVelocity =
                    loader.getDoubleValue(
                            ConstantsLoader.combinePath(
                                    base_steps, "motion_magic", "cruise_velocity"));
            fxsConfig.MotionMagic.MotionMagicAcceleration =
                    loader.getDoubleValue(
                            ConstantsLoader.combinePath(
                                    base_steps, "motion_magic", "acceleration"));
            fxsConfig.MotionMagic.MotionMagicJerk =
                    loader.getDoubleValue(
                            ConstantsLoader.combinePath(base_steps, "motion_magic", "jerk"));
        }

        // Load a supply current limit if configured
        if (loader.getBoolValue(
                ConstantsLoader.combinePath(base_steps, "supply_limit", "enabled"))) {
            fxsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            fxsConfig.CurrentLimits.SupplyCurrentLimit =
                    loader.getDoubleValue(
                            ConstantsLoader.combinePath(
                                    base_steps, "supply_limit", "current_limit"));
            fxsConfig.CurrentLimits.SupplyCurrentLowerTime =
                    loader.getDoubleValue(
                            ConstantsLoader.combinePath(
                                    base_steps, "supply_limit", "trigger_time"));
        }

        // Load a stator current limit if configured
        if (loader.getBoolValue(
                ConstantsLoader.combinePath(base_steps, "stator_limit", "enabled"))) {
            fxsConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            fxsConfig.CurrentLimits.StatorCurrentLimit =
                    loader.getDoubleValue(
                            ConstantsLoader.combinePath(
                                    base_steps, "stator_limit", "current_limit"));
        }
    }
}
