package com.marswars.mechanisms.spark;

import com.marswars.util.ConstantsLoader;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SparkMotorConfig {

    protected final ConstantsLoader loader = ConstantsLoader.getInstance();

    public enum SparkMotorType {
        VORTEX,
        NEO_550,
        PULSAR_775,
        BRUSHED
    }

    public String canbus_name = "rio";

    public int can_id = 1;

    public SparkMotorType motor_type = SparkMotorType.VORTEX;

    public SparkBaseConfig config;

    public SparkMotorConfig(SparkMotorConfig config) {
        this.canbus_name = config.canbus_name;
        this.can_id = config.can_id;
        this.motor_type = config.motor_type;
        this.config = config.config;
    }

    public SparkMotorConfig() {}

    public void loadFromConfig(String... base_steps) {
        canbus_name = loader.getStringValue(ConstantsLoader.combinePath(base_steps, "bus_name"));
        can_id = loader.getIntValue(ConstantsLoader.combinePath(base_steps, "bus_id"));

        String motor_type_str =
                loader.getStringValue(ConstantsLoader.combinePath(base_steps, "type"));
        if (motor_type_str.equals("VORTEX")) {
            motor_type = SparkMotorType.VORTEX;
        } else if (motor_type_str.equals("NEO_550")) {
            motor_type = SparkMotorType.NEO_550;
        } else if (motor_type_str.equals("PULSAR_775")) {
            motor_type = SparkMotorType.PULSAR_775;
        } else if (motor_type_str.equals("BRUSHED")) {
            motor_type = SparkMotorType.BRUSHED;
        } else {
            throw new RuntimeException("Unknown motor type: " + motor_type_str);
        }

        // Load the base motor configs
        config.inverted(loader.getBoolValue(ConstantsLoader.combinePath(base_steps, "inverted")));
                     
        config.idleMode(
                loader.getBoolValue(ConstantsLoader.combinePath(base_steps, "brake_mode"))
                        ? IdleMode.kBrake
                        : IdleMode.kCoast);

        // Load the slot configs
        FeedForwardConfig feedForwardConfig = config.closedLoop.feedForward;
        ClosedLoopConfig closedLoopConfig = config.closedLoop;
        feedForwardConfig.kS(
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "ks")), ClosedLoopSlot.kSlot0);
        feedForwardConfig.kV(
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kv")), ClosedLoopSlot.kSlot0);
        feedForwardConfig.kA(
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "ka")), ClosedLoopSlot.kSlot0);
        feedForwardConfig.kG(
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kg")), ClosedLoopSlot.kSlot0);
        closedLoopConfig.p(
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kp")), ClosedLoopSlot.kSlot0);
        closedLoopConfig.i(
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "ki")), ClosedLoopSlot.kSlot0);
        closedLoopConfig.d(
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot0", "kd")), ClosedLoopSlot.kSlot0);
        feedForwardConfig.kS(
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "ks")), ClosedLoopSlot.kSlot1);
        feedForwardConfig.kV(
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kv")), ClosedLoopSlot.kSlot1);
        feedForwardConfig.kA(
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "ka")), ClosedLoopSlot.kSlot1);
        feedForwardConfig.kG(
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kg")), ClosedLoopSlot.kSlot1);
        closedLoopConfig.p(
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kp")), ClosedLoopSlot.kSlot1);
        closedLoopConfig.i(
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "ki")), ClosedLoopSlot.kSlot1);
        closedLoopConfig.d(
                loader.getDoubleValue(ConstantsLoader.combinePath(base_steps, "slot1", "kd")), ClosedLoopSlot.kSlot1);
        closedLoopConfig.apply(feedForwardConfig);

        // Load the motion magic configs
        MAXMotionConfig motionConfig = config.closedLoop.maxMotion;
        if (loader.getBoolValue(
                ConstantsLoader.combinePath(base_steps, "motion_magic", "enabled"))) {
            motionConfig.cruiseVelocity(
                    loader.getDoubleValue(
                            ConstantsLoader.combinePath(
                                    base_steps, "motion_magic", "cruise_velocity")));
            motionConfig.maxAcceleration(
                    loader.getDoubleValue(
                            ConstantsLoader.combinePath(
                                    base_steps, "motion_magic", "acceleration")));
        }
        closedLoopConfig.apply(motionConfig);
        config.apply(closedLoopConfig);

        // Load a supply current limit if configured
        if (loader.getBoolValue(
                ConstantsLoader.combinePath(base_steps, "supply_limit", "enabled"))) {
            config.smartCurrentLimit(
                    (int)loader.getDoubleValue(
                            ConstantsLoader.combinePath(
                                    base_steps, "supply_limit", "current_limit")));
        }
    }
}
