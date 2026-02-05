package com.marswars.mechanisms;

import java.util.List;

import com.marswars.util.FxMotorConfig;

public class TurretMech extends RollerMech {
    /**
     * Constructs a new TurretMech
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs Configuration for the turret motor
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param moi Moment of inertia of the turret in kg*m^2 (Simulation only)
     */
    public TurretMech(
            String logging_prefix,
            List<FxMotorConfig> motor_configs,
            double gear_ratio,
            double moi){
        super(logging_prefix, null, motor_configs, gear_ratio, moi);
    }

    /**
     * Constructs a new TurretMech
     *
     * @param logging_prefix String prefix for logging
     * @param mech_name Name of the mechanism
     * @param motor_configs Configuration for the turret motor
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param moi Moment of inertia of the turret in kg*m^2 (Simulation only)
     */
    public TurretMech(
            String logging_prefix,
            String mech_name,
            List<FxMotorConfig> motor_configs,
            double gear_ratio,
            double moi){
        super(logging_prefix, mech_name, motor_configs, gear_ratio, moi);
    }
}
