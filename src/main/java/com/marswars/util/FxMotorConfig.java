package frc.mw_lib.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public class FxMotorConfig {

    protected final ConstantsLoader loader = ConstantsLoader.getInstance();

    public enum FxMotorType {
        X60,
        X44,
        FALCON500
    }

    public String canbus_name = "rio";

    public int can_id = 1;

    public FxMotorType motor_type = FxMotorType.X60;

    public boolean use_motion_magic = false;

    public TalonFXConfiguration config = new TalonFXConfiguration();

    public void loadFromConfig(String... base_steps) {
        canbus_name = loader.getStringValue(ConstantsLoader.combinePath(base_steps, "bus_name"));
        can_id = loader.getIntValue(ConstantsLoader.combinePath(base_steps, "bus_id"));

        String motor_type_str = loader.getStringValue(ConstantsLoader.combinePath(base_steps, "type"));
        if (motor_type_str.equals("X60")) {
            motor_type = FxMotorType.X60;
        } else if (motor_type_str.equals("X44")) {
            motor_type = FxMotorType.X44;
        } else {
            throw new RuntimeException("Unknown motor type: " + motor_type_str);
        }

        // Load the base motor configs
        config.MotorOutput.Inverted = loader.getBoolValue(ConstantsLoader.combinePath(base_steps, "inverted"))
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = loader.getBoolValue(ConstantsLoader.combinePath(base_steps, "brake_mode"))
                ? com.ctre.phoenix6.signals.NeutralModeValue.Brake
                : com.ctre.phoenix6.signals.NeutralModeValue.Coast;

        // Load the slot configs
        config.Slot0.kS = loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "ks"));
        config.Slot0.kV = loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kv"));
        config.Slot0.kA = loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "ka"));
        config.Slot0.kG = loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kg"));
        config.Slot0.kP = loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kp"));
        config.Slot0.kI = loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "ki"));
        config.Slot0.kD = loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kd"));
        config.Slot1.kS = loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "ks"));
        config.Slot1.kV = loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kv"));
        config.Slot1.kA = loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "ka"));
        config.Slot1.kG = loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kg"));
        config.Slot1.kP = loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kp"));
        config.Slot1.kI = loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "ki"));
        config.Slot1.kD = loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kd"));

        // Load the motion magic configs
        if (loader.getBoolValue(ConstantsLoader.combinePath(base_steps, "motion_magic", "enabled"))) {
            use_motion_magic = true;
            config.MotionMagic.MotionMagicCruiseVelocity = loader
                    .getDoubleValue(ConstantsLoader.combinePath(base_steps, "motion_magic", "cruise_velocity"));
            config.MotionMagic.MotionMagicAcceleration = loader
                    .getDoubleValue(ConstantsLoader.combinePath(base_steps, "motion_magic", "acceleration"));
            config.MotionMagic.MotionMagicJerk = loader
                    .getDoubleValue(ConstantsLoader.combinePath(base_steps, "motion_magic", "jerk"));
        }

        // Load a supply current limit if configured
        if (loader.getBoolValue(ConstantsLoader.combinePath(base_steps, "supply_limit", "enabled"))) {
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = loader
                    .getDoubleValue(ConstantsLoader.combinePath(base_steps, "supply_limit", "current_limit"));
            config.CurrentLimits.SupplyCurrentLowerTime = loader
                    .getDoubleValue(ConstantsLoader.combinePath(base_steps, "supply_limit", "trigger_time"));
        }

        // Load a stator current limit if configured
        if( loader.getBoolValue(ConstantsLoader.combinePath(base_steps, "stator_limit", "enabled"))) {
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = loader
                    .getDoubleValue(ConstantsLoader.combinePath(base_steps, "stator_limit", "current_limit"));
        }

    }

}
