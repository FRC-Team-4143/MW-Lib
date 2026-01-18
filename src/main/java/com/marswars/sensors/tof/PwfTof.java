package com.marswars.sensors.tof;

import com.playingwithfusion.TimeOfFlight;

public class PwfTof extends Tof {

    final TimeOfFlight sensor_;

    /**
     * Creates a PWF Time-of-Flight sensor interface
     * @param logging_prefix logging prefix for the sensor
     * @param can_id can id of the sensor
     * @param mode ranging mode
     * @param sample_time sample time in seconds
     */
    public PwfTof(String logging_prefix, int can_id, RangeMode mode, double sample_time){
        super(logging_prefix, can_id);
        sensor_ = new TimeOfFlight(can_id);


        TimeOfFlight.RangingMode tof_mode;
        switch (mode) {
            case SHORT:
                tof_mode = TimeOfFlight.RangingMode.Short;
                break;
            case MEDIUM:
                tof_mode = TimeOfFlight.RangingMode.Medium;
                break;
            case LONG:
                tof_mode = TimeOfFlight.RangingMode.Long;
                break;
            default:
                tof_mode = TimeOfFlight.RangingMode.Medium;
                break;
        }

        sensor_.setRangingMode(tof_mode, sample_time * 1000); // convert to milliseconds
    }

    @Override
    public void readInputs(double timestamp) {
        if(!IS_SIM){
            range_ = sensor_.getRange() / 1000.0; // convert mm to meters
        } else {
            // Let external function set the range in simulation
        }
    }
    
}
