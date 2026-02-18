package com.marswars.mechanisms;

import com.marswars.mechanisms.fx.FxRollerMech;
import com.marswars.subsystem.SubsystemIoBase;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.NovaMotorConfig;
import java.util.List;

/**
 * Generic roller mechanism that automatically selects the correct implementation
 * based on the motor configuration type provided.
 */
public class RollerMech implements SubsystemIoBase {

    private final SubsystemIoBase delegate_;

    public RollerMech(String logging_prefix, List<?> motor_configs, double gear_ratio) {
        this(logging_prefix, null, motor_configs, gear_ratio, 0.00001);
    }

    public RollerMech(String logging_prefix, String mech_name, List<?> motor_configs, double gear_ratio) {
        this(logging_prefix, mech_name, motor_configs, gear_ratio, 0.00001);
    }

    public RollerMech(String logging_prefix, List<?> motor_configs, double gear_ratio, double roller_inertia) {
        this(logging_prefix, null, motor_configs, gear_ratio, roller_inertia);
    }

    @SuppressWarnings("unchecked")
    public RollerMech(String logging_prefix, String mech_name, List<?> motor_configs, double gear_ratio, double roller_inertia) {
        if (motor_configs == null || motor_configs.isEmpty()) {
            throw new IllegalArgumentException("motor_configs cannot be null or empty");
        }

        Object firstConfig = motor_configs.get(0);
        
        if (firstConfig instanceof FxMotorConfig) {
            delegate_ = new FxRollerMech(
                    logging_prefix, mech_name, (List<FxMotorConfig>) motor_configs, 
                    gear_ratio, roller_inertia);
        } else if (firstConfig instanceof NovaMotorConfig) {
            throw new UnsupportedOperationException(
                    "NovaRollerMech is not yet implemented.");
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
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxRollerMech) {
            ((com.marswars.mechanisms.fx.FxRollerMech) delegate_).setTargetPosition(position_rad);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetPosition");
        }
    }

    public void setTargetVelocity(double velocity_rad_per_sec) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxRollerMech) {
            ((com.marswars.mechanisms.fx.FxRollerMech) delegate_).setTargetVelocity(velocity_rad_per_sec);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetVelocity");
        }
    }

    public void setTargetDutyCycle(double duty_cycle) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxRollerMech) {
            ((com.marswars.mechanisms.fx.FxRollerMech) delegate_).setTargetDutyCycle(duty_cycle);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetDutyCycle");
        }
    }

    public double getCurrentPosition() {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxRollerMech) {
            return ((com.marswars.mechanisms.fx.FxRollerMech) delegate_).getCurrentPosition();
        } else {
            throw new UnsupportedOperationException("Delegate does not support getCurrentPosition");
        }
    }

    public double getCurrentVelocity() {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxRollerMech) {
            return ((com.marswars.mechanisms.fx.FxRollerMech) delegate_).getCurrentVelocity();
        } else {
            throw new UnsupportedOperationException("Delegate does not support getCurrentVelocity");
        }
    }

    public void setCurrentPosition(double position_rad) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxRollerMech) {
            ((com.marswars.mechanisms.fx.FxRollerMech) delegate_).setCurrentPosition(position_rad);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setCurrentPosition");
        }
    }

    public double getLeaderCurrent() {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxRollerMech) {
            return ((com.marswars.mechanisms.fx.FxRollerMech) delegate_).getLeaderCurrent();
        } else {
            throw new UnsupportedOperationException("Delegate does not support getLeaderCurrent");
        }
    }

    public void applyLoadTorque(double torque_nm) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxRollerMech) {
            ((com.marswars.mechanisms.fx.FxRollerMech) delegate_).applyLoadTorque(torque_nm);
        } else {
            throw new UnsupportedOperationException("Delegate does not support applyLoadTorque");
        }
    }
}
