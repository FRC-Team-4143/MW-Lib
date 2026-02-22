package com.marswars.mechanisms.nova;

import com.marswars.util.ConstantsLoader;
import com.thethriftybot.devices.ThriftyNova.CurrentType;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig;

import edu.wpi.first.wpilibj.DataLogManager;

public class NovaMotorConfig {

    protected final ConstantsLoader loader = ConstantsLoader.getInstance();

    public enum NovaMotorType {
        VORTEX,
        NEO_550,
        PULSAR_775,
        MINION,
    }

    public String canbus_name = "rio";

    public int can_id = 1;

    public NovaMotorType motor_type = NovaMotorType.VORTEX;

    private boolean supply_current_limit_enabled = false;
    
    public ThriftyNovaConfig config = new ThriftyNovaConfig();

    public NovaMotorConfig(NovaMotorConfig config) {
        this.canbus_name = config.canbus_name;
        this.can_id = config.can_id;
        this.motor_type = config.motor_type;
        this.config = config.config;
    }

    public NovaMotorConfig() {}

    public void loadFromConfig(String... base_steps) {
        canbus_name = loader.getStringValue(ConstantsLoader.combinePath(base_steps, "bus_name"));
        can_id = loader.getIntValue(ConstantsLoader.combinePath(base_steps, "bus_id"));

        String motor_type_str =
                loader.getStringValue(ConstantsLoader.combinePath(base_steps, "type"));
        if (motor_type_str.equals("VORTEX")) {
            motor_type = NovaMotorType.VORTEX;
        } else if (motor_type_str.equals("NEO_550")) {
            motor_type = NovaMotorType.NEO_550;
        } else if (motor_type_str.equals("PULSAR_775")) {
            motor_type = NovaMotorType.PULSAR_775;
        } else if (motor_type_str.equals("MINION")) {
            motor_type = NovaMotorType.MINION;
        } else {
            throw new RuntimeException("Unknown motor type: " + motor_type_str);
        }

        // Load the base motor configs
        config.inverted =
                loader.getBoolValue(ConstantsLoader.combinePath(base_steps, "inverted"));
        config.brakeMode =
                loader.getBoolValue(ConstantsLoader.combinePath(base_steps, "brake_mode"));

        // Load the slot configs
        config.pid0.f =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "ks"));
        config.pid0.p =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kp"));
        config.pid0.i =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "ki"));
        config.pid0.d =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kd"));
        config.pid1.f =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "ks"));
        config.pid1.p =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kp"));
        config.pid1.i =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "ki"));
        config.pid1.d =
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kd"));

        // Load a supply current limit if configured
        if (loader.getBoolValue(
                ConstantsLoader.combinePath(base_steps, "supply_limit", "enabled"))) {
            config.currentType = CurrentType.SUPPLY;
            supply_current_limit_enabled = true;
            config.maxCurrent = loader.getDoubleValue(
                    ConstantsLoader.combinePath(
                            base_steps, "supply_limit", "current_limit"));
        }

        // Load a stator current limit if configured
        if (loader.getBoolValue(
                ConstantsLoader.combinePath(base_steps, "stator_limit", "enabled"))) {
            if(supply_current_limit_enabled) {
                DataLogManager.log("Cannot configure supply and current limit on Nova - only one can be enabled. Stator limit will take precedence.");
                supply_current_limit_enabled = false;
            }
            config.currentType = CurrentType.STATOR;
            config.maxCurrent = loader.getDoubleValue(
                    ConstantsLoader.combinePath(base_steps, "stator_limit", "current_limit"));
        }
    }
}
