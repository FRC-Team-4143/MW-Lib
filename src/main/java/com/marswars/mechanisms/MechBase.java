package frc.mw_lib.mechanisms;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.RobotBase;
import frc.mw_lib.subsystem.SubsystemIoBase;
import frc.mw_lib.util.FxMotorConfig;

public abstract class MechBase implements SubsystemIoBase {

    public class ConstructedMotors {
        public TalonFX motors[];
        public BaseStatusSignal signals[];
    }

    private final String mech_name_;
    private String logging_prefix_ = "Subsystem/Unknown/";

    protected final boolean IS_SIM;

    public MechBase(String logging_prefix) {
        // Identify the mecahnism name
        String name = this.getClass().getSimpleName();
        name = name.substring(name.lastIndexOf('.') + 1);
        if (name.endsWith("Mech")) {
            name = name.substring(0, name.length() - "Mech".length());
        }
        mech_name_ = name;

        // identiy if we are in simulation
        IS_SIM = RobotBase.isSimulation();
        setLoggingPrefix(logging_prefix);
    }

    public void setLoggingPrefix(String subsystem_name) {
        logging_prefix_ = subsystem_name;
    }

    public String getLoggingKey() {
        return logging_prefix_ + mech_name_ + "/";
    }

    public String getMechName() {
        return mech_name_;
    }

    public ConstructedMotors configMotors(List<FxMotorConfig> motor_configs, double sensor_to_mech_ratio,
            Function<FxMotorConfig, FxMotorConfig> configMaster) {
        // throw a fit if we don't have any motors
        if (motor_configs == null || motor_configs.size() == 0) {
            throw new IllegalArgumentException("Motor configs is null or empty");
        }
        ConstructedMotors constructed = new ConstructedMotors();
        List<BaseStatusSignal> all_signals_list = new ArrayList<>();
        constructed.motors = new TalonFX[motor_configs.size()];
        for (int i = 0; i < motor_configs.size(); i++) {
            FxMotorConfig cfg = motor_configs.get(i);
            if (cfg.canbus_name == null || cfg.canbus_name.isEmpty()) {
                throw new IllegalArgumentException("Motor canbus name is null or empty");
            }

            constructed.motors[i] = new TalonFX(cfg.can_id, cfg.canbus_name);
            ArrayList<BaseStatusSignal> motor_signals = new ArrayList<>();

            // Only apply the configs to the first motor, the rest are followers
            if (i == 0) {
                if (configMaster != null) {
                    cfg = configMaster.apply(cfg);
                }

                // also force the gear ratio to be correct
                cfg.config.Feedback.SensorToMechanismRatio = sensor_to_mech_ratio;

                constructed.motors[i].getConfigurator().apply(cfg.config);

                motor_signals.add(constructed.motors[i].getPosition());
                motor_signals.add(constructed.motors[i].getVelocity());
            } else {
                // make the rest of the motors followers
                constructed.motors[i].setControl(new StrictFollower(constructed.motors[0].getDeviceID()));
            }

            motor_signals.add(constructed.motors[i].getMotorVoltage());
            motor_signals.add(constructed.motors[i].getSupplyCurrent());
            motor_signals.add(constructed.motors[i].getDeviceTemp());
            // motor_signals.add(constructed.motors[i].getSupplyVoltage()); // skip
            // refreshing voltage
            // to keep bandwidth low

            // Optimize bus usage to the signals we want
            for (BaseStatusSignal s : motor_signals) {
                s.setUpdateFrequency(50); // 50 Hz update rate
            }
            constructed.motors[i].optimizeBusUtilization();

            // keep a master list of signals for refreshing later
            all_signals_list.addAll(motor_signals);
        }

        // convert the list to an array for easy access
        constructed.signals = new BaseStatusSignal[all_signals_list.size()];
        constructed.signals = all_signals_list.toArray(constructed.signals);

        return constructed;
    }

    public ConstructedMotors configMotors(List<FxMotorConfig> motor_configs, double sensor_to_mech_ratio) {
        return configMotors(motor_configs, sensor_to_mech_ratio, null);
    }   

}
