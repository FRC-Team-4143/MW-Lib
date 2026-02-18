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

    // Delegation methods for common operations
    public void setTargetPosition(double position_rad) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            ((com.marswars.mechanisms.fx.FxArmMech) delegate_).setTargetPosition(position_rad);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            ((com.marswars.mechanisms.nova.NovaArmMech) delegate_).setTargetPosition(position_rad);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetPosition");
        }
    }

    public void setTargetVelocity(double velocity_rad_per_sec) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            ((com.marswars.mechanisms.fx.FxArmMech) delegate_).setTargetVelocity(velocity_rad_per_sec);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            ((com.marswars.mechanisms.nova.NovaArmMech) delegate_).setTargetVelocity(velocity_rad_per_sec);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetVelocity");
        }
    }

    public void setTargetDutyCycle(double duty_cycle) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            ((com.marswars.mechanisms.fx.FxArmMech) delegate_).setTargetDutyCycle(duty_cycle);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            ((com.marswars.mechanisms.nova.NovaArmMech) delegate_).setTargetDutyCycle(duty_cycle);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetDutyCycle");
        }
    }

    public double getCurrentPosition() {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            return ((com.marswars.mechanisms.fx.FxArmMech) delegate_).getCurrentPosition();
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            return ((com.marswars.mechanisms.nova.NovaArmMech) delegate_).getCurrentPosition();
        } else {
            throw new UnsupportedOperationException("Delegate does not support getCurrentPosition");
        }
    }

    public double getCurrentVelocity() {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            return ((com.marswars.mechanisms.fx.FxArmMech) delegate_).getCurrentVelocity();
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            return ((com.marswars.mechanisms.nova.NovaArmMech) delegate_).getCurrentVelocity();
        } else {
            throw new UnsupportedOperationException("Delegate does not support getCurrentVelocity");
        }
    }

    public void setCurrentPosition(double position_rad) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            ((com.marswars.mechanisms.fx.FxArmMech) delegate_).setCurrentPosition(position_rad);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            ((com.marswars.mechanisms.nova.NovaArmMech) delegate_).setCurrentPosition(position_rad);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setCurrentPosition");
        }
    }
}
