package com.marswars.mechanisms.spark;

import com.ctre.phoenix6.BaseStatusSignal;
import com.marswars.mechanisms.MechBase;
import com.marswars.mechanisms.spark.SparkMotorConfig.SparkMotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * TalonFX-specific mechanism base class.
 * Provides motor configuration helpers for CTRE TalonFX motor controllers.
 */
public abstract class SparkMechBase extends MechBase<SparkBase, SparkMotorConfig> {

    /**
     * Creates a new Spark-based mechanism
     * @param logging_prefix the logging prefix for this mechanism
     */
    public SparkMechBase(String logging_prefix) {
        super(logging_prefix);
    }

    /**
     * Creates a new TalonFX-based mechanism with a specified mechanism name
     * @param logging_prefix the logging prefix for this mechanism
     * @param mech_name the name of the mechanism (used for multiple instances of the same mech)
     */
    public SparkMechBase(String logging_prefix, String mech_name) {
        super(logging_prefix, mech_name);
    }

    /**
     * Configures the TalonFX motors based on the given motor configs
     *
     * @param motor_configs the list of motor configs
     * @param sensor_to_mech_ratio the sensor to mechanism ratio
     * @param configMaster a function to modify the master motor config before applying it
     * @return the constructed motors and their signals
     */
    @Override
    public ConstructedMotors configMotors(
            List<SparkMotorConfig> motor_configs,
            double sensor_to_mech_ratio,
            Function<SparkMotorConfig, SparkMotorConfig> configMaster) {
        // throw a fit if we don't have any motors
        if (motor_configs == null || motor_configs.size() == 0) {
            throw new IllegalArgumentException("Motor configs is null or empty");
        }
        ConstructedMotors constructed = new ConstructedMotors();
        constructed.motors = new SparkBase[motor_configs.size()];
        for (int i = 0; i < motor_configs.size(); i++) {
            SparkMotorConfig cfg = motor_configs.get(i);
            if (cfg.canbus_name == null || cfg.canbus_name.isEmpty()) {
                throw new IllegalArgumentException("Motor canbus name is null or empty");
            }

            if(cfg.motor_type == SparkMotorType.VORTEX){
                constructed.motors[i] = new SparkFlex(cfg.can_id, MotorType.kBrushless);
            } else if (cfg.motor_type == SparkMotorType.BRUSHED) {
                constructed.motors[i] = new SparkMax(cfg.can_id, MotorType.kBrushed);
            } else {
                constructed.motors[i] = new SparkMax(cfg.can_id, MotorType.kBrushless);
            }

            if (configMaster != null) {
                cfg = configMaster.apply(cfg);
            }

            // Make follower motors (non-master motors)
            if (i > 0) {
                cfg.config.follow(constructed.motors[0].getDeviceId());
            }

            // Apply the configs to the motor
            constructed.motors[i].configure(cfg.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        return constructed;
    }

    /**
     * Helper method to get ClosedLoopConfigAccessor from a SparkBase by casting to the concrete type.
     * This is useful when working with SparkBase arrays but needing access to configAccessor.
     *
     * @param motor The SparkBase motor
     * @return The ClosedLoopConfigAccessor
     * @throws IllegalArgumentException if the motor is not SparkMax or SparkFlex
     */
    protected static ClosedLoopConfigAccessor getClosedLoopConfigAccessor(SparkBase motor) {
        if (motor instanceof SparkMax sparkMax) {
            return sparkMax.configAccessor.closedLoop;
        } else if (motor instanceof SparkFlex sparkFlex) {
            return sparkFlex.configAccessor.closedLoop;
        }
        throw new IllegalArgumentException("Motor must be SparkMax or SparkFlex, got: " + motor.getClass().getName());
    }
}
