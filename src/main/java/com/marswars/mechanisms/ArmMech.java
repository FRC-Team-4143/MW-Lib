package com.marswars.mechanisms;

import com.marswars.mechanisms.fx.FxArmMech;
import com.marswars.mechanisms.fx.FxMotorConfig;
import com.marswars.mechanisms.nova.NovaArmMech;
import com.marswars.mechanisms.nova.NovaMotorConfig;
import com.marswars.subsystem.SubsystemIoBase;

import java.util.List;

/**
 * Generic arm mechanism that automatically selects the correct implementation
 * based on the motor configuration type provided.
 */
public class ArmMech implements SubsystemIoBase {

    private final SubsystemIoBase delegate_;

    /**
     * Constructs a new ArmMech with gravity compensation enabled by default
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs  List of motor configurations (FxMotorConfig or NovaMotorConfig)
     * @param gear_ratio     Gear ratio as motor rotations / mechanism rotations
     * @param arm_length     Length of the arm in meters (Simulation only)
     * @param arm_mass       Mass of the arm in kg (Simulation only)
     * @param min_angle      Minimum angle of the arm in radians (Simulation only)
     * @param max_angle      Maximum angle of the arm in radians (Simulation only)
     */
    public ArmMech(String logging_prefix, List<?> motor_configs, double gear_ratio, double arm_length, double arm_mass, double min_angle, double max_angle) {
        this(logging_prefix, null, motor_configs, gear_ratio, arm_length, arm_mass, min_angle, max_angle, true);
    }

    /**
     * Constructs a new ArmMech with a mechanism name and gravity compensation enabled by default
     *
     * @param logging_prefix String prefix for logging
     * @param mech_name      Name of the mechanism (used for multiple instances of the same mech)
     * @param motor_configs  List of motor configurations (FxMotorConfig or NovaMotorConfig)
     * @param gear_ratio     Gear ratio as motor rotations / mechanism rotations
     * @param arm_length     Length of the arm in meters (Simulation only)
     * @param arm_mass       Mass of the arm in kg (Simulation only)
     * @param min_angle      Minimum angle of the arm in radians (Simulation only)
     * @param max_angle      Maximum angle of the arm in radians (Simulation only)
     */
    public ArmMech(String logging_prefix, String mech_name, List<?> motor_configs, double gear_ratio, double arm_length, double arm_mass, double min_angle, double max_angle) {
        this(logging_prefix, mech_name, motor_configs, gear_ratio, arm_length, arm_mass, min_angle, max_angle, true);
    }

    /**
     * Constructs a new ArmMech with configurable gravity compensation
     *
     * @param logging_prefix     String prefix for logging
     * @param motor_configs      List of motor configurations (FxMotorConfig or NovaMotorConfig)
     * @param gear_ratio         Gear ratio as motor rotations / mechanism rotations
     * @param arm_length         Length of the arm in meters (Simulation only)
     * @param arm_mass           Mass of the arm in kg (Simulation only)
     * @param min_angle          Minimum angle of the arm in radians (Simulation only)
     * @param max_angle          Maximum angle of the arm in radians (Simulation only)
     * @param gravity_compensate true to enable gravity compensation, false otherwise
     */
    public ArmMech(String logging_prefix, List<?> motor_configs, double gear_ratio, double arm_length, double arm_mass, double min_angle, double max_angle, boolean gravity_compensate) {
        this(logging_prefix, null, motor_configs, gear_ratio, arm_length, arm_mass, min_angle, max_angle, gravity_compensate);
    }

    /**
     * Constructs a new ArmMech with a mechanism name and configurable gravity compensation
     *
     * @param logging_prefix     String prefix for logging
     * @param mech_name          Name of the mechanism (used for multiple instances of the same mech)
     * @param motor_configs      List of motor configurations (FxMotorConfig or NovaMotorConfig)
     * @param gear_ratio         Gear ratio as motor rotations / mechanism rotations
     * @param arm_length         Length of the arm in meters (Simulation only)
     * @param arm_mass           Mass of the arm in kg (Simulation only)
     * @param min_angle          Minimum angle of the arm in radians (Simulation only)
     * @param max_angle          Maximum angle of the arm in radians (Simulation only)
     * @param gravity_compensate true to enable gravity compensation, false otherwise
     */
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

    /** {@inheritDoc} */
    @Override
    public void readInputs(double timestamp) {
        delegate_.readInputs(timestamp);
    }

    /** {@inheritDoc} */
    @Override
    public void writeOutputs(double timestamp) {
        delegate_.writeOutputs(timestamp);
    }

    /** {@inheritDoc} */
    @Override
    public void logData() {
        delegate_.logData();
    }

    /**
     * Gets the underlying delegate mechanism implementation
     *
     * @return the delegate mechanism (FxArmMech or NovaArmMech)
     */
    public SubsystemIoBase getDelegate() {
        return delegate_;
    }

    /**
     * Set the target position of the arm using standard position control
     *
     * @param position_rad the target position in radians
     */
    public void setTargetPosition(double position_rad) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            ((com.marswars.mechanisms.fx.FxArmMech) delegate_).setTargetPosition(position_rad);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            ((com.marswars.mechanisms.nova.NovaArmMech) delegate_).setTargetPosition(position_rad);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetPosition");
        }
    }

    /**
     * Set the target velocity of the arm
     *
     * @param velocity_rad_per_sec the target velocity in radians per second
     */
    public void setTargetVelocity(double velocity_rad_per_sec) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            ((com.marswars.mechanisms.fx.FxArmMech) delegate_).setTargetVelocity(velocity_rad_per_sec);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            ((com.marswars.mechanisms.nova.NovaArmMech) delegate_).setTargetVelocity(velocity_rad_per_sec);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetVelocity");
        }
    }

    /**
     * Set the target duty cycle of the arm
     *
     * @param duty_cycle the target duty cycle (-1.0 to 1.0)
     */
    public void setTargetDutyCycle(double duty_cycle) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            ((com.marswars.mechanisms.fx.FxArmMech) delegate_).setTargetDutyCycle(duty_cycle);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            ((com.marswars.mechanisms.nova.NovaArmMech) delegate_).setTargetDutyCycle(duty_cycle);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetDutyCycle");
        }
    }

    /**
     * Get the current position of the arm
     *
     * @return the current position in radians
     */
    public double getCurrentPosition() {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            return ((com.marswars.mechanisms.fx.FxArmMech) delegate_).getCurrentPosition();
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            return ((com.marswars.mechanisms.nova.NovaArmMech) delegate_).getCurrentPosition();
        } else {
            throw new UnsupportedOperationException("Delegate does not support getCurrentPosition");
        }
    }

    /**
     * Get the current velocity of the arm
     *
     * @return the current velocity in radians per second
     */
    public double getCurrentVelocity() {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            return ((com.marswars.mechanisms.fx.FxArmMech) delegate_).getCurrentVelocity();
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            return ((com.marswars.mechanisms.nova.NovaArmMech) delegate_).getCurrentVelocity();
        } else {
            throw new UnsupportedOperationException("Delegate does not support getCurrentVelocity");
        }
    }

    /**
     * Set the current position of the arm (for zeroing)
     *
     * @param position_rad the current position in radians
     */
    public void setCurrentPosition(double position_rad) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            ((com.marswars.mechanisms.fx.FxArmMech) delegate_).setCurrentPosition(position_rad);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            ((com.marswars.mechanisms.nova.NovaArmMech) delegate_).setCurrentPosition(position_rad);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setCurrentPosition");
        }
    }

    /**
     * Set the target position of the arm with arbitrary feed forward.
     * This allows additional control output while holding a position.
     *
     * @param position_rad          the target position in radians
     * @param arbitrary_feedforward arbitrary feed forward value (volts for Nova, arbitrary units for FX depending on slot gains configuration)
     */
    public void setTargetPositionWithFF(double position_rad, double arbitrary_feedforward) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            ((com.marswars.mechanisms.fx.FxArmMech) delegate_).setTargetPositionWithFF(position_rad, arbitrary_feedforward);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            ((com.marswars.mechanisms.nova.NovaArmMech) delegate_).setTargetPositionWithFF(position_rad, arbitrary_feedforward);
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetPositionWithFF");
        }
    }

    /**
     * Set the target position of the arm using motion profile control
     *
     * @param position_rad the target position in radians
     * @throws UnsupportedOperationException if using NovaArmMech (not supported)
     */
    public void setTargetPositionMotionProfile(double position_rad) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            ((com.marswars.mechanisms.fx.FxArmMech) delegate_).setTargetPositionMotionProfile(position_rad);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            throw new UnsupportedOperationException("NovaArmMech does not support Motion Profile");
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetPositionMotionProfile");
        }
    }

    /**
     * Set the target position of the arm with arbitrary feed forward using motion profile control.
     * This allows additional control output while holding a position.
     *
     * @param position_rad          the target position in radians
     * @param arbitrary_feedforward arbitrary feed forward value (units depend on slot gains configuration)
     * @throws UnsupportedOperationException if using NovaArmMech (not supported)
     */
    public void setTargetPositionMotionProfileWithFF(double position_rad, double arbitrary_feedforward) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            ((com.marswars.mechanisms.fx.FxArmMech) delegate_).setTargetPositionMotionProfileWithFF(position_rad, arbitrary_feedforward);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            throw new UnsupportedOperationException("NovaArmMech does not support Motion Profile");
        } else {
            throw new UnsupportedOperationException("Delegate does not support setTargetPositionMotionProfileWithFF");
        }
    }

    /**
     * Get the current draw of the leader motor
     *
     * @return the current draw in amps
     */
    public double getLeaderCurrent() {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            return ((com.marswars.mechanisms.fx.FxArmMech) delegate_).getLeaderCurrent();
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            return ((com.marswars.mechanisms.nova.NovaArmMech) delegate_).getLeaderCurrent();
        } else {
            throw new UnsupportedOperationException("Delegate does not support getLeaderCurrent");
        }
    }

    /**
     * Applies a load torque to the arm mechanism for simulation purposes.
     * This method should be called during the simulation update cycle to apply
     * external loads (like friction, compression forces, etc.) to the mechanism.
     *
     * @param torque_nm The load torque in Newton-meters (Nm) at the arm output shaft.
     *                  Positive values oppose motion in the positive direction.
     */
    public void applyLoadTorque(double torque_nm) {
        if (delegate_ instanceof com.marswars.mechanisms.fx.FxArmMech) {
            ((com.marswars.mechanisms.fx.FxArmMech) delegate_).applyLoadTorque(torque_nm);
        } else if (delegate_ instanceof com.marswars.mechanisms.nova.NovaArmMech) {
            ((com.marswars.mechanisms.nova.NovaArmMech) delegate_).applyLoadTorque(torque_nm);
        } else {
            throw new UnsupportedOperationException("Delegate does not support applyLoadTorque");
        }
    }
}
