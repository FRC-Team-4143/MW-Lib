package frc.mw_lib.mechanisms;

import edu.wpi.first.wpilibj.RobotBase;
import frc.mw_lib.subsystem.SubsystemIoBase;

public abstract class MechBase implements SubsystemIoBase {
    private final String mech_name_;
    private String logging_prefix_ = "Subsystem/Unknown/";

    protected final boolean IS_SIM;

    public MechBase(){
        // Identify the mecahnism name
        String name = this.getClass().getSimpleName();
        name = name.substring(name.lastIndexOf('.') + 1);
        if (name.endsWith("Mech")) {
          name = name.substring(0, name.length() - "Mech".length());
        }
        mech_name_ = name;

        // identiy if we are in simulation
        IS_SIM = RobotBase.isSimulation();
    }

    public void setLoggingPrefix(String subsystem_name){
        logging_prefix_ = subsystem_name;
    }

    public String getLoggingKey(){
        return logging_prefix_  + mech_name_ + "/";
    }

    public String getMechName(){
        return mech_name_;
    }
    
}
