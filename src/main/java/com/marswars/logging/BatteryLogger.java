package com.marswars.logging;

import com.playingwithfusion.BattFuelGauge;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.math.filter.Debouncer;

public class BatteryLogger {
    private static BattFuelGauge bfg_ = null;

    private static StringPublisher nickname_pub_ =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/BATT_NICKNAME").publish();

    private static final Alert batteryDisconnectedAlert_ = 
        new Alert("Battery Fuel Gauge is not connected to CAN network.", AlertType.kInfo);
    
    private static final Debouncer connectionDebouncer_ = new Debouncer(0.5);
    private static boolean connected_ = false;

    public static void logBatteryData() {
        try {
            // Initialize the battery fuel gauge if not already done
            if (bfg_ == null) {
                bfg_ = new BattFuelGauge(0);
                nickname_pub_.set(bfg_.getNickname());
            }
            
            // Try to read data from the battery fuel gauge
            double chargePercent = bfg_.getRemainingChargePct();
            double voltage = bfg_.getVoltage();
            double current = bfg_.getCurrent();
            
            // If we successfully read data, the device is connected
            // Check for valid data (non-zero voltage is a good indicator)
            boolean isConnected = voltage > 0.0;
            connected_ = connectionDebouncer_.calculate(isConnected);
            batteryDisconnectedAlert_.set(!connected_);
            
            if (connected_) {
                DogLog.log("BFG/Charge Percent", chargePercent);
                DogLog.log("BFG/Voltage", voltage);
                DogLog.log("BFG/Current", current);
                DogLog.log("BFG/Connected", true);
            } else {
                DogLog.log("BFG/Connected", false);
            }
        } catch (Exception e) {
            // If we get an exception, the device is likely not connected
            connected_ = connectionDebouncer_.calculate(false);
            batteryDisconnectedAlert_.set(!connected_);
            DogLog.log("BFG/Connected", false);
        }
    }
}
