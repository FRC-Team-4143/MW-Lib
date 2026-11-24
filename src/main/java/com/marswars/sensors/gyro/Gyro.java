package com.marswars.sensors.gyro;

import com.marswars.logging.MwLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

import com.marswars.subsystem.ControllableIo;
import com.marswars.subsystem.IoControls;
import org.littletonrobotics.junction.Logger;

public abstract class Gyro implements ControllableIo {

    private final Alert gyroDisconnectedAlert;
    private final Debouncer connection_debouncer_ = new Debouncer(0.5);

    private final String gyro_name_;
    private String logging_prefix_;

    protected final boolean IS_SIM;

    protected final GyroInputsAutoLogged inputs_ = new GyroInputsAutoLogged();

    /** Set once the owning subsystem is registered; null until then. */
    private IoControls controls_ = null;

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

    /** {@inheritDoc} */
    @Override
    public String getIoName() {
        return gyro_name_;
    }

    /**
     * {@inheritDoc} Yaw, pitch and roll can be overridden as {@code Yaw}, {@code Pitch} and {@code
     * Roll}, in degrees.
     */
    @Override
    public void attachControls(IoControls controls) {
        controls_ = controls;
        controls.declareSensor("Yaw");
        controls.declareSensor("Pitch");
        controls.declareSensor("Roll");
    }

    /** @return false while this gyro is switched off from the dashboard */
    public boolean isAvailable() {
        return controls_ == null || controls_.isEnabled();
    }

    /**
     * @return true if the gyro is connected (or its yaw is overridden) and not switched off; code
     *     should fall back (e.g. swerve uses kinematics) when false
     */
    public boolean isConnected() {
        if (!isAvailable()) {
            return false;
        }
        return inputs_.connectedDebounced
                || (controls_ != null && controls_.isSensorOverridden("Yaw"));
    }

    public void setYaw(Rotation2d yaw) {}

    public Rotation2d getYawPosition() {
        return angle("Yaw", inputs_.yawPosition);
    }

    public Rotation2d getPitchPosition() {
        return angle("Pitch", inputs_.pitchPosition);
    }

    public Rotation2d getRollPosition() {
        return angle("Roll", inputs_.rollPosition);
    }

    /** The reading, its override, or zero while switched off. */
    private Rotation2d angle(String channel, Rotation2d raw) {
        if (!isAvailable()) {
            return Rotation2d.kZero;
        }
        if (controls_ == null || !controls_.isSensorOverridden(channel)) {
            return raw;
        }
        return Rotation2d.fromDegrees(controls_.sensorValue(channel, raw.getDegrees()));
    }

    public double getYawVelocityRadPerSec() {
        return isAvailable() ? inputs_.yawVelocityRadPerSec : 0.0;
    }

    @Override
    public void logData() {
    }

    @Override
    public void writeOutputs(double timestamp) {
        // no outputs to write
    }
}
