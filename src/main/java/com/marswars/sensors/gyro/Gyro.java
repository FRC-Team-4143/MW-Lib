package com.marswars.sensors.gyro;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

import com.marswars.subsystem.SubsystemIoBase;

public abstract class Gyro implements SubsystemIoBase {

    private final Alert gyroDisconnectedAlert;

    private final String gyro_name_;
    private String logging_prefix_;

    protected boolean connected = false;
    protected Rotation2d yawPosition = new Rotation2d();
    protected double yawVelocityRadPerSec = 0.0;

    protected final boolean IS_SIM;
    
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
        readGyro();
        gyroDisconnectedAlert.set(!connected);
    }

    public abstract void readGyro();

    public boolean isConnected() {
        return connected;
    }

    public void setYaw(Rotation2d yaw) {}

    public Rotation2d getYawPosition() {
        return yawPosition;
    }

    public double getYawVelocityRadPerSec() {
        return yawVelocityRadPerSec;
    }

    @Override
    public void logData() {
        DogLog.log(getLoggingKey() + "Connected", connected);
        DogLog.log(getLoggingKey() + "YawPositionDeg", yawPosition.getDegrees());
        DogLog.log(getLoggingKey() + "YawVelocityRadPerSec", yawVelocityRadPerSec);
    }

    @Override
    public void writeOutputs(double timestamp) {
        // no outputs to write
    }
}
