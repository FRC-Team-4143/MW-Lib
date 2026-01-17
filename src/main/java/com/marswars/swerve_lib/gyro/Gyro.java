package com.marswars.swerve_lib.gyro;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import com.marswars.mechanisms.MechBase;

public abstract class Gyro extends MechBase {

    private final Alert gyroDisconnectedAlert;

    protected boolean connected = false;
    protected Rotation2d yawPosition = new Rotation2d();
    protected double yawVelocityRadPerSec = 0.0;

    public Gyro(String logging_prefix) {
        super(logging_prefix);
        gyroDisconnectedAlert =
                new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
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
