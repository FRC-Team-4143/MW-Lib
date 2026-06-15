package com.marswars.sensors.gyro;

import com.marswars.logging.MwLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

import com.marswars.subsystem.SubsystemIoBase;
import org.littletonrobotics.junction.Logger;

public abstract class Gyro implements SubsystemIoBase {

    private final Alert gyroDisconnectedAlert;
    private final Debouncer connection_debouncer_ = new Debouncer(0.5);

    private final String gyro_name_;
    private String logging_prefix_;

    protected final boolean IS_SIM;

    protected final GyroInputsAutoLogged inputs_ = new GyroInputsAutoLogged();

    public Gyro(String logging_prefix) {
        // Identify the mecahnism name
        String name = this.getClass().getSimpleName();
        name = name.substring(name.lastIndexOf('.') + 1);
        if (name.endsWith("Gyro")) {
            name = name.substring(0, name.length() - "Gyro".length());
        }
        gyro_name_ = name;

        // identiy if we are in simulation
        IS_SIM = RobotBase.isSimulation();
        setLoggingPrefix(logging_prefix);

        gyroDisconnectedAlert =
                new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
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
        return logging_prefix_ + gyro_name_ + "/";
    }

    @Override
    public void readInputs(double timestamp) {
        if (!MwLog.isReplay()) {
            readGyro();
            inputs_.connectedDebounced = connection_debouncer_.calculate(inputs_.connected);
        }
        Logger.processInputs(getLoggingKey() + "Inputs", inputs_);
        gyroDisconnectedAlert.set(IS_SIM ? false : !inputs_.connectedDebounced);
    }

    public abstract void readGyro();

    public boolean isConnected() {
        return inputs_.connectedDebounced;
    }

    public void setYaw(Rotation2d yaw) {}

    public Rotation2d getYawPosition() {
        return inputs_.yawPosition;
    }

    public Rotation2d getPitchPosition() {
        return inputs_.pitchPosition;
    }

    public Rotation2d getRollPosition() {
        return inputs_.rollPosition;
    }

    public double getYawVelocityRadPerSec() {
        return inputs_.yawVelocityRadPerSec;
    }

    @Override
    public void logData() {
        MwLog.log(getLoggingKey() + "Connected", inputs_.connectedDebounced);
        MwLog.log(getLoggingKey() + "YawPositionDeg", inputs_.yawPosition.getDegrees());
        MwLog.log(getLoggingKey() + "PitchPositionDeg", inputs_.pitchPosition.getDegrees());
        MwLog.log(getLoggingKey() + "RollPositionDeg", inputs_.rollPosition.getDegrees());
        MwLog.log(getLoggingKey() + "YawVelocityRadPerSec", inputs_.yawVelocityRadPerSec);
    }

    @Override
    public void writeOutputs(double timestamp) {
        // no outputs to write
    }
}
