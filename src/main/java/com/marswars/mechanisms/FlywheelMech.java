package com.marswars.mechanisms;

import com.marswars.mechanisms.fx.FxFlywheelMech;
import com.marswars.mechanisms.nova.NovaFlywheelMech;
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
            delegate_ = new NovaFlywheelMech(
                    logging_prefix, mech_name, (List<NovaMotorConfig>) motor_configs, 
                    gear_ratio, wheel_inertia, wheel_radius);
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
    
    /**
     * Sets the target velocity of the flywheel in radians per second using standard velocity control
     *
     * @param velocity_rad_per_sec the target velocity in radians per second
     */
    public void setTargetVelocity(double velocity_rad_per_sec) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxFlywheelMech) {
            ((com.marswars.mechanisms.fx.FxFlywheelMech) delegate_).setTargetVelocity(velocity_rad_per_sec);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaFlywheelMech) {
            ((com.marswars.mechanisms.nova.NovaFlywheelMech) delegate_).setTargetVelocity(velocity_rad_per_sec);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetVelocity");
        }
    }

    /**
     * Sets the target velocity of the flywheel with feed forward.
     * This allows additional control output while maintaining velocity.
     *
     * @param velocity_rad_per_sec  the target velocity in radians per second
     * @param arbitrary_feedforward feedforward value (volts for Nova, arbitrary units for FX depending on slot gains configuration)
     */
    public void setTargetVelocityWithFF(double velocity_rad_per_sec, double arbitrary_feedforward) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxFlywheelMech) {
            ((com.marswars.mechanisms.fx.FxFlywheelMech) delegate_).setTargetVelocityWithFF(velocity_rad_per_sec, arbitrary_feedforward);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaFlywheelMech) {
            ((com.marswars.mechanisms.nova.NovaFlywheelMech) delegate_).setTargetVelocityWithFF(velocity_rad_per_sec, arbitrary_feedforward);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetVelocityWithFF");
        }
    }

    /**
     * Sets the target velocity of the flywheel in radians per second using motion profile velocity control
     *
     * @param velocity_rad_per_sec the target velocity in radians per second
     */
    public void setTargetVelocityMotionProfile(double velocity_rad_per_sec) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxFlywheelMech) {
            ((com.marswars.mechanisms.fx.FxFlywheelMech) delegate_).setTargetVelocityMotionProfile(velocity_rad_per_sec);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaFlywheelMech) {
            ((com.marswars.mechanisms.nova.NovaFlywheelMech) delegate_).setTargetVelocityMotionProfile(velocity_rad_per_sec);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetVelocityMotionProfile");
        }
    }

    /**
     * Sets the target duty cycle of the flywheel
     *
     * @param duty_cycle the target duty cycle (-1.0 to 1.0)
     */
    public void setTargetDutyCycle(double duty_cycle) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxFlywheelMech) {
            ((com.marswars.mechanisms.fx.FxFlywheelMech) delegate_).setTargetDutyCycle(duty_cycle);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaFlywheelMech) {
            ((com.marswars.mechanisms.nova.NovaFlywheelMech) delegate_).setTargetDutyCycle(duty_cycle);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetDutyCycle");
        }
    }

    /**
     * @return The current velocity of the flywheel in radians per second
     */
    public double getCurrentVelocity() {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxFlywheelMech) {
            return ((com.marswars.mechanisms.fx.FxFlywheelMech) delegate_).getCurrentVelocity();
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaFlywheelMech) {
            return ((com.marswars.mechanisms.nova.NovaFlywheelMech) delegate_).getCurrentVelocity();
        } else {
            throw new UnsupportedOperationException("Delegate does not support getCurrentVelocity");
        }
    }

    /**
     * Applies a load torque to the flywheel mechanism for simulation purposes.
     * This method should be called during the simulation update cycle to apply
     * external loads (like friction, compression forces, etc.) to the mechanism.
     *
     * @param torque_nm The load torque in Newton-meters (Nm) at the flywheel output shaft.
     *                  Positive values oppose motion in the positive direction.
     */
    public void applyLoadTorque(double torque_nm) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxFlywheelMech) {
            ((com.marswars.mechanisms.fx.FxFlywheelMech) delegate_).applyLoadTorque(torque_nm);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaFlywheelMech) {
            ((com.marswars.mechanisms.nova.NovaFlywheelMech) delegate_).applyLoadTorque(torque_nm);
        } else {
            throw new UnsupportedOperationException("Delegate does not support applyLoadTorque");
        }
    }
}
