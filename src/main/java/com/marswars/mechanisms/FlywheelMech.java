package com.marswars.mechanisms;

import com.marswars.mechanisms.fx.FxFlywheelMech;
import com.marswars.subsystem.SubsystemIoBase;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.NovaMotorConfig;
import java.util.List;

/**
 * Generic flywheel mechanism that automatically selects the correct implementation
 * based on the motor configuration type provided.
 */
public class FlywheelMech implements SubsystemIoBase {

    private final SubsystemIoBase delegate_;

    public FlywheelMech(
            String logging_prefix,
            List<?> motor_configs,
            double gear_ratio,
            double wheel_inertia,
            double wheel_radius) {
        this(logging_prefix, null, motor_configs, gear_ratio, wheel_inertia, wheel_radius);
    }

    @SuppressWarnings("unchecked")
    public FlywheelMech(
            String logging_prefix,
            String mech_name,
            List<?> motor_configs,
            double gear_ratio,
            double wheel_inertia,
            double wheel_radius) {
        if (motor_configs == null || motor_configs.isEmpty()) {
            throw new IllegalArgumentException("motor_configs cannot be null or empty");
        }

        Object firstConfig = motor_configs.get(0);
        
        if (firstConfig instanceof FxMotorConfig) {
            delegate_ = new FxFlywheelMech(
                    logging_prefix, mech_name, (List<FxMotorConfig>) motor_configs, 
                    gear_ratio, wheel_inertia, wheel_radius);
        } else if (firstConfig instanceof NovaMotorConfig) {
            throw new UnsupportedOperationException(
                    "NovaFlywheelMech is not yet implemented. Use FxFlywheelMech directly for TalonFX motors.");
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
