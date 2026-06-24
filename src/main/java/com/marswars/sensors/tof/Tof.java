package com.marswars.sensors.tof;

import com.marswars.subsystem.SubsystemIoBase;

import com.marswars.logging.MwLog;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

public abstract class Tof implements SubsystemIoBase{

    protected final String tof_name_;

    protected final boolean IS_SIM;

    private String logging_prefix_ = "Tof/Unknown/";

    protected final TofInputsAutoLogged inputs_ = new TofInputsAutoLogged();

    public enum RangeMode {
        SHORT,
        MEDIUM,
        LONG
    }

    Tof(String logging_prefix, int id){
        // Identify the mecahnism name
        String name = this.getClass().getSimpleName();
        name = name.substring(name.lastIndexOf('.') + 1);
        if (name.endsWith("Tof")) {
            name = name.substring(0, name.length() - "Tof".length());
        }
        tof_name_ = name + id;

        // identiy if we are in simulation
        IS_SIM = RobotBase.isSimulation();
        setLoggingPrefix(logging_prefix);
        if(IS_SIM){
            MwLog.tunable(getLoggingKey() + "Range Override", 0.0, (val) -> inputs_.range = val);
        }
    }

    private void setLoggingPrefix(String subsystem_name) {
        logging_prefix_ = subsystem_name;
    }

    /**
     * Get the logging key for this mechanism
     *
     * @return the logging key
     */
    public String getLoggingKey() {
        return logging_prefix_ + tof_name_ + "/";
    }

    @Override
    public void readInputs(double timestamp) {
        if (!MwLog.isReplay()) {
            readTof();
        }
        Logger.processInputs(getLoggingKey() + "Inputs", inputs_);
    }

    protected abstract void readTof();

    @Override
    public void writeOutputs(double timestamp) {
        // Sensors typically do not have outputs to write
    }

    @Override
    public void logData() {
        MwLog.log(getLoggingKey() + "Range", inputs_.range);
    }

    /**
     * Get the measured range in meters
     * @return range in meters
     */
    public double getRange(){
        return inputs_.range;
    }

    /**
     * Set the measured range in meters (for simulation only)
     * @param range range in meters
     */
    public void setRange(double range){
        if(IS_SIM){
            inputs_.range = range;
        }
    }
}
