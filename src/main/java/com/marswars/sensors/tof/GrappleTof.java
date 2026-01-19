package com.marswars.sensors.tof;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

public class GrappleTof extends Tof {

    final LaserCan sensor_;
    Measurement measurement_  = null;

    /**
     * Creates a Grapple Robotics Time-of-Flight sensor interface
     * @param logging_prefix logging prefix for the sensor
     * @param can_id can id of the sensor
     * @param mode ranging mode
     * @param sample_time sample time in seconds
     */
    public GrappleTof(String logging_prefix, int can_id, RangeMode mode, double sample_time){
        super(logging_prefix, can_id);
        sensor_ = new LaserCan(can_id);

        LaserCanInterface.RangingMode tof_mode;
        switch (mode) {
            case SHORT:
                tof_mode = LaserCanInterface.RangingMode.SHORT;
                break;
            case MEDIUM:
                tof_mode = LaserCanInterface.RangingMode.SHORT;
                break;
            case LONG:
                tof_mode = LaserCanInterface.RangingMode.LONG;
                break;
            default:
                tof_mode = LaserCanInterface.RangingMode.SHORT;
                break;
        }

        TimingBudget timing_budget;
        if(sample_time >= 0.1){
            timing_budget = TimingBudget.TIMING_BUDGET_100MS;
        } else if(sample_time >= 0.05){
            timing_budget = TimingBudget.TIMING_BUDGET_50MS;
        } else if(sample_time >= 0.033){
            timing_budget = TimingBudget.TIMING_BUDGET_33MS;
        } else {
            timing_budget = TimingBudget.TIMING_BUDGET_20MS;
        }

        try {
            sensor_.setRangingMode(tof_mode);
            sensor_.setTimingBudget(timing_budget);
        } catch (au.grapplerobotics.ConfigurationFailedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void readInputs(double timestamp) {
        if(!IS_SIM){
            measurement_ = sensor_.getMeasurement();
            if(measurement_ != null && measurement_.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
                range_ = measurement_.distance_mm / 1000.0; // convert mm to meters
            } else {
                range_ = -1.0; // invalid range
            }
        } else {
            // Let external function set the range in simulation
        }
    }
    
}
