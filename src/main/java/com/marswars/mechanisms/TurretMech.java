package com.marswars.mechanisms;

import java.util.List;

import com.marswars.util.FxMotorConfig;

public class TurretMech extends RollerMech {
    TurretMech(
            String logging_prefix,
            List<FxMotorConfig> motor_configs,
            double gear_ratio,
            double moi){
        super(logging_prefix, motor_configs, gear_ratio, moi);
    }
}
