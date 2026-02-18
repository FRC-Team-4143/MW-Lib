package com.marswars.mechanisms;

import com.marswars.mechanisms.fx.FxArmMech;
import com.marswars.mechanisms.nova.NovaArmMech;
import com.marswars.subsystem.SubsystemIoBase;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.NovaMotorConfig;
import java.util.List;

/**
 * Generic arm mechanism that automatically selects the correct implementation
 * based on the motor configuration type provided.
 */
public class ArmMech implements SubsystemIoBase {

    private final SubsystemIoBase delegate_;

    public ArmMech(String logging_prefix, List<?> motor_configs, double gear_ratio, double arm_length, double arm_mass, double min_angle, double max_angle) {
        this(logging_prefix, null, motor_configs, gear_ratio, arm_length, arm_mass, min_angle, max_angle, true);
    }

    public ArmMech(String logging_prefix, String mech_name, List<?> motor_configs, double gear_ratio, double arm_length, double arm_mass, double min_angle, double max_angle) {
        this(logging_prefix, mech_name, motor_configs, gear_ratio, arm_length, arm_mass, min_angle, max_angle, true);
    }

    public ArmMech(String logging_prefix, List<?> motor_configs, double gear_ratio, double arm_length, double arm_mass, double min_angle, double max_angle, boolean gravity_compensate) {
        this(logging_prefix, null, motor_configs, gear_ratio, arm_length, arm_mass, min_angle, max_angle, gravity_compensate);
    }

    @SuppressWarnings("unchecked")
    public ArmMech(String logging_prefix, String mech_name, List<?> motor_configs, double gear_ratio, double arm_length, double arm_mass, double min_angle, double max_angle, boolean gravity_compensate) {
        if (motor_configs == null || motor_configs.isEmpty()) {
            throw new IllegalArgumentException("motor_configs cannot be null or empty");
        }

        Object firstConfig = motor_configs.get(0);
        
        if (firstConfig instanceof FxMotorConfig) {
            delegate_ = new FxArmMech(
                    logging_prefix, mech_name, (List<FxMotorConfig>) motor_configs, 
                    gear_ratio, arm_length, arm_mass, min_angle, max_angle, gravity_compensate);
        } else if (firstConfig instanceof NovaMotorConfig) {
            delegate_ = new NovaArmMech(
                    logging_prefix, mech_name, (List<NovaMotorConfig>) motor_configs,
                    gear_ratio, arm_length, arm_mass, min_angle, max_angle, gravity_compensate);
        } else {
            throw new IllegalArgumentException(
                    "Unsupported motor config type: " + firstConfig.getClass().getName());
        }
    }

    @Override
    public void readInputs(double timestamp) {
        delegate_.readInputs(timestamp);
    }

    @Override
    public void writeOutputs(double timestamp) {
        delegate_.writeOutputs(timestamp);
    }

    @Override
    public void logData() {
        delegate_.logData();
    }

    public SubsystemIoBase getDelegate() {
        return delegate_;
    }
}
