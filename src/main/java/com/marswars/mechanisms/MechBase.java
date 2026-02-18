package com.marswars.mechanisms;

import edu.wpi.first.wpilibj.RobotBase;
import com.marswars.subsystem.SubsystemIoBase;
import java.util.List;
import java.util.function.Function;

/**
 * Base class for mechanism IO implementations with common motor configuration helpers.
 * This is a generic template class that can be specialized for different motor controller types.
 *
 * @param <TMotor> The motor controller type (e.g., TalonFX, ThriftyNova)
 * @param <TConfig> The motor configuration type (e.g., FxMotorConfig, NovaMotorConfig)
 */
public abstract class MechBase<TMotor, TConfig> implements SubsystemIoBase {

    /** Container for constructed motor controllers. */
    public class ConstructedMotors {
        public TMotor motors[];
    }

    private String mech_name_;
    private String logging_prefix_ = "Subsystem/Unknown/";

    protected final boolean IS_SIM;

    /**
     * Creates a new mechanism base
     * @param logging_prefix the logging prefix for this mechanism
     */
    public MechBase(String logging_prefix) {
        this(logging_prefix, null);
    }

    /**
     * Creates a new mechanism base with a specified mechanism name
     * @param logging_prefix the logging prefix for this mechanism
     * @param mech_name the name of the mechanism (used for mutliple instances of the same mech)
     */
    public MechBase(String logging_prefix, String mech_name) {
        // Identify the mecahnism name
        if(mech_name == null){
            String name = this.getClass().getSimpleName();
            name = name.substring(name.lastIndexOf('.') + 1);
            if (name.endsWith("Mech")) {
                name = name.substring(0, name.length() - "Mech".length());
            }
            mech_name_ = name;
        } else {
            mech_name_ = mech_name;
        }
        

        // identiy if we are in simulation
        IS_SIM = RobotBase.isSimulation();
        logging_prefix_ = logging_prefix;
    }

    /**
     * Get the logging key for this mechanism.
     *
     * @return the logging key
     */
    public String getLoggingKey() {
        return logging_prefix_ + mech_name_ + "/";
    }

    /**
     * Get the name of the mechanism.
     *
     * @return the name of the mechanism
     */
    public String getMechName() {
        return mech_name_;
    }

    /**
     * Configures the motors based on the given motor configs.
     * Subclasses must implement this to handle motor-specific configuration.
     *
     * @param motor_configs the list of motor configs
     * @param sensor_to_mech_ratio the sensor to mechanism ratio
     * @param configMaster a function to modify the master motor config before applying it
     * @return the constructed motors and their signals
     */
    public ConstructedMotors configMotors(
            List<TConfig> motor_configs,
            double sensor_to_mech_ratio,
            Function<TConfig, TConfig> configMaster) {
        throw new UnsupportedOperationException(
                "configMotors not implemented for " + getClass().getSimpleName() +
                ". If you're extending MechBase without using configMotors, consider using FxMechBase or NovaMechBase.");
    }

    /**
     * Configures the motors based on the given motor configs.
     *
     * @param motor_configs the list of motor configs
     * @param sensor_to_mech_ratio the sensor to mechanism ratio
     * @return the constructed motors and their signals
     */
    public ConstructedMotors configMotors(
            List<TConfig> motor_configs, double sensor_to_mech_ratio) {
        return configMotors(motor_configs, sensor_to_mech_ratio, null);
    }
}
