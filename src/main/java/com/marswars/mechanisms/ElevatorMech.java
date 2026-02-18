package com.marswars.mechanisms;

import com.marswars.mechanisms.fx.FxElevatorMech;
import com.marswars.subsystem.SubsystemIoBase;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.NovaMotorConfig;
import java.util.List;

/**
 * Generic elevator mechanism that automatically selects the correct implementation
 * based on the motor configuration type provided.
 */
public class ElevatorMech implements SubsystemIoBase {

    private final SubsystemIoBase delegate_;

    public ElevatorMech(String logging_prefix, List<?> motor_configs, double gear_ratio, double drum_radius, double carriage_mass, double min_height, double max_height, boolean is_vertical) {
        this(logging_prefix, null, motor_configs, gear_ratio, drum_radius, carriage_mass, min_height, max_height, is_vertical);
    }

    @SuppressWarnings("unchecked")
    public ElevatorMech(String logging_prefix, String mech_name, List<?> motor_configs, double gear_ratio, double drum_radius, double carriage_mass, double min_height, double max_height, boolean is_vertical) {
        if (motor_configs == null || motor_configs.isEmpty()) {
            throw new IllegalArgumentException("motor_configs cannot be null or empty");
        }

        Object firstConfig = motor_configs.get(0);
        
        if (firstConfig instanceof FxMotorConfig) {
            delegate_ = new FxElevatorMech(
                    logging_prefix, mech_name, (List<FxMotorConfig>) motor_configs, 
                    gear_ratio, drum_radius, carriage_mass, min_height, max_height, is_vertical);
        } else if (firstConfig instanceof NovaMotorConfig) {
            throw new UnsupportedOperationException(
                    "NovaElevatorMech is not yet implemented.");
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
    public void setTargetPosition(double position_m) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxElevatorMech) {
            ((com.marswars.mechanisms.fx.FxElevatorMech) delegate_).setTargetPosition(position_m);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetPosition");
        }
    }

    public void setTargetPositionMotionProfile(double position_m) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxElevatorMech) {
            ((com.marswars.mechanisms.fx.FxElevatorMech) delegate_).setTargetPositionMotionProfile(position_m);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetPositionMotionProfile");
        }
    }

    public void setTargetPositionWithFF(double position_m, double arbitrary_feedforward) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxElevatorMech) {
            ((com.marswars.mechanisms.fx.FxElevatorMech) delegate_).setTargetPositionWithFF(position_m, arbitrary_feedforward);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetPositionWithFF");
        }
    }

    public void setTargetPositionMotionProfileWithFF(double position_m, double arbitrary_feedforward) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxElevatorMech) {
            ((com.marswars.mechanisms.fx.FxElevatorMech) delegate_).setTargetPositionMotionProfileWithFF(position_m, arbitrary_feedforward);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetPositionMotionProfileWithFF");
        }
    }

    public void setTargetVelocity(double velocity_mps) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxElevatorMech) {
            ((com.marswars.mechanisms.fx.FxElevatorMech) delegate_).setTargetVelocity(velocity_mps);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetVelocity");
        }
    }

    public void setTargetDutyCycle(double duty_cycle) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxElevatorMech) {
            ((com.marswars.mechanisms.fx.FxElevatorMech) delegate_).setTargetDutyCycle(duty_cycle);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetDutyCycle");
        }
    }

    public double getCurrentPosition() {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxElevatorMech) {
            return ((com.marswars.mechanisms.fx.FxElevatorMech) delegate_).getCurrentPosition();
        } else {
            throw new UnsupportedOperationException("Delegate does not support getCurrentPosition");
        }
    }

    public double getCurrentVelocity() {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxElevatorMech) {
            return ((com.marswars.mechanisms.fx.FxElevatorMech) delegate_).getCurrentVelocity();
        } else {
            throw new UnsupportedOperationException("Delegate does not support getCurrentVelocity");
        }
    }

    public void setCurrentPosition(double position_m) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxElevatorMech) {
            ((com.marswars.mechanisms.fx.FxElevatorMech) delegate_).setCurrentPosition(position_m);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setCurrentPosition");
        }
    }

    public double getLeaderCurrent() {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxElevatorMech) {
            return ((com.marswars.mechanisms.fx.FxElevatorMech) delegate_).getLeaderCurrent();
        } else {
            throw new UnsupportedOperationException("Delegate does not support getLeaderCurrent");
        }
    }

    public void applyLoadTorque(double torque_nm) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxElevatorMech) {
            ((com.marswars.mechanisms.fx.FxElevatorMech) delegate_).applyLoadTorque(torque_nm);
        } else {
            throw new UnsupportedOperationException("Delegate does not support applyLoadTorque");
        }
    }
}
