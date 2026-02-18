package com.marswars.mechanisms.fx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.marswars.mechanisms.MechBase;
import com.marswars.util.FxMotorConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * TalonFX-specific mechanism base class.
 * Provides motor configuration helpers for CTRE TalonFX motor controllers.
 */
public abstract class FxMechBase extends MechBase<TalonFX, FxMotorConfig> {

    /** Container for constructed TalonFX motor controllers and their status signals. */
    public class ConstructedFxMotors extends ConstructedMotors {
        public BaseStatusSignal signals[];
    }

    /**
     * Creates a new TalonFX-based mechanism
     * @param logging_prefix the logging prefix for this mechanism
     */
    public FxMechBase(String logging_prefix) {
        super(logging_prefix);
    }

    /**
     * Creates a new TalonFX-based mechanism with a specified mechanism name
     * @param logging_prefix the logging prefix for this mechanism
     * @param mech_name the name of the mechanism (used for multiple instances of the same mech)
     */
    public FxMechBase(String logging_prefix, String mech_name) {
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
    public ConstructedFxMotors configMotors(
            List<FxMotorConfig> motor_configs,
            double sensor_to_mech_ratio,
            Function<FxMotorConfig, FxMotorConfig> configMaster) {
        // throw a fit if we don't have any motors
        if (motor_configs == null || motor_configs.size() == 0) {
            throw new IllegalArgumentException("Motor configs is null or empty");
        }
        ConstructedFxMotors constructed = new ConstructedFxMotors();
        List<BaseStatusSignal> all_signals_list = new ArrayList<>();
        constructed.motors = new TalonFX[motor_configs.size()];
        for (int i = 0; i < motor_configs.size(); i++) {
            FxMotorConfig cfg = motor_configs.get(i);
            if (cfg.canbus_name == null || cfg.canbus_name.isEmpty()) {
                throw new IllegalArgumentException("Motor canbus name is null or empty");
            }

            constructed.motors[i] = new TalonFX(cfg.can_id, cfg.canbus_name);
            ArrayList<BaseStatusSignal> motor_signals = new ArrayList<>();

            if (configMaster != null) {
                cfg = configMaster.apply(cfg);
            }

            // also force the gear ratio to be correct
            cfg.config.Feedback.SensorToMechanismRatio = sensor_to_mech_ratio;
            // Apply the configs to the motor
            constructed.motors[i].getConfigurator().apply(cfg.config);

            // Only register signals for the master motor (the first one) to avoid duplicates and save bandwidth
            if (i == 0) {
                motor_signals.add(constructed.motors[i].getPosition());
                motor_signals.add(constructed.motors[i].getVelocity());
            } else {
                // make the rest of the motors followers
                constructed.motors[i].setControl(
                        new StrictFollower(constructed.motors[0].getDeviceID()));
            }

            motor_signals.add(constructed.motors[i].getMotorVoltage());
            motor_signals.add(constructed.motors[i].getSupplyCurrent());
            motor_signals.add(constructed.motors[i].getDeviceTemp());

            // Optimize bus usage to the signals we want
            for (BaseStatusSignal s : motor_signals) {
                s.setUpdateFrequency(50); // 50 Hz update rate
            }
            constructed.motors[i].optimizeBusUtilization();

            // keep a master list of signals for refreshing later
            all_signals_list.addAll(motor_signals);
        }

        // convert the list to an array for easy access
        constructed.signals = new BaseStatusSignal[all_signals_list.size()];
        constructed.signals = all_signals_list.toArray(constructed.signals);

        return constructed;
    }
}
