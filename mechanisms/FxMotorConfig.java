package frc.mw_lib.mechanisms;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class FxMotorConfig {

    public enum FxMotorType {
        X60,
        X44
    }

    public String canbus_name = "rio";

    public int can_id = 1;

    public FxMotorType motor_type = FxMotorType.X60;

    public TalonFXConfiguration config = new TalonFXConfiguration();
    
}
