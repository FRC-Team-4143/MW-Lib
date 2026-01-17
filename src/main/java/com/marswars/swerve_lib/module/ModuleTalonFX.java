package com.marswars.swerve_lib.module;

import static edu.wpi.first.units.Units.Radians;
import static com.marswars.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogEncoder;
import com.marswars.swerve_lib.PhoenixOdometryThread;
import com.marswars.util.MWPreferences;
import com.marswars.util.PhoenixUtil;
import java.util.Arrays;
import org.ejml.simple.UnsupportedOperation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

public class ModuleTalonFX extends Module {
    protected final TalonFX drive_talonfx_;
    protected final TalonFX steer_talonfx_;

    // pick one of any of these encoders based on your configuration
    protected final CANcoder cancoder;
    protected final AnalogEncoder encoder;

    // Non FOC control requests
    protected final VoltageOut voltage_req_ = new VoltageOut(0);
    protected final PositionVoltage position_voltage_req_ = new PositionVoltage(0.0);
    protected final VelocityVoltage velocity_voltage_req_ = new VelocityVoltage(0.0);

    // Torque-current control requests
    protected final TorqueCurrentFOC torque_current_req_ = new TorqueCurrentFOC(0);
    protected final PositionTorqueCurrentFOC position_torque_current_req_ =
            new PositionTorqueCurrentFOC(0.0);
    protected final VelocityTorqueCurrentFOC velocity_torque_current_req_ =
            new VelocityTorqueCurrentFOC(0.0);

    // Inputs from drive motor
    protected final StatusSignal<Angle> drive_position_sig_;
    protected final StatusSignal<AngularVelocity> drive_velocity_sig_;
    protected final StatusSignal<Voltage> drive_applied_volts_sig_;
    protected final StatusSignal<Current> drive_current_sig_;

    // Inputs from steer motor
    protected final StatusSignal<Angle> steer_absolute_position_sig_;
    protected final StatusSignal<AngularVelocity> steer_velocity_sig_;
    protected final StatusSignal<Voltage> steer_applied_volts_sig_;
    protected final StatusSignal<Current> steer_current_sig_;

    public ModuleTalonFX(
            String logging_prefix,
            int index,
            SwerveModuleConfig config,
            SwerveModuleSimulation simulation) {
        super(logging_prefix, index, config, simulation);

        drive_talonfx_ =
                new TalonFX(
                        config_.drive_motor_config.can_id, config_.drive_motor_config.canbus_name);
        steer_talonfx_ =
                new TalonFX(
                        config_.steer_motor_config.can_id, config_.steer_motor_config.canbus_name);

        if (config_.encoder_type != SwerveModuleConfig.EncoderType.ANALOG_ENCODER) {
            cancoder = new CANcoder(config_.encoder_id, config_.drive_motor_config.canbus_name);
            encoder = null;
        } else {
            cancoder = null;
            encoder = new AnalogEncoder(config_.encoder_id);
        }

        // Configure drive motor
        tryUntilOk(
                5,
                () ->
                        drive_talonfx_
                                .getConfigurator()
                                .apply(config_.drive_motor_config.config, 0.25));
        tryUntilOk(5, () -> drive_talonfx_.setPosition(0.0, 0.25));

        // Configure steer motor
        var steerConfig = config_.steer_motor_config.config;
        steerConfig.Feedback.SensorToMechanismRatio = config_.module_type.steerRatio;
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        // config the encoder offset for the steer motor
        if (config_.encoder_type == SwerveModuleConfig.EncoderType.ANALOG_ENCODER) {
            // Use analog encoder
            Rotation2d encoder_value = Rotation2d.fromRotations(encoder.get());
            Rotation2d encoder_offset =
                    Rotation2d.fromRotations(
                            MWPreferences.getInstance()
                                    .getPreferenceDouble("Encoder" + index + "Offset", 0.0));
            Rotation2d zero_position = encoder_value.minus(encoder_offset);
            steer_talonfx_.setPosition(zero_position.getRotations());
        } else if (config_.encoder_type == SwerveModuleConfig.EncoderType.CTRE_CAN_CODER) {
            // Use remote CANCoder
            // TODO CJT implement for cancoder
            throw new UnsupportedOperation("CANCoder not yet supported in ModuleTalonFX");
        } else {
            throw new IllegalArgumentException("Unsupported encoder type for ModuleTalonFX");
        }

        steerConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / config_.module_type.steerRatio;
        steerConfig.MotionMagic.MotionMagicAcceleration =
                steerConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        steerConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * config_.module_type.steerRatio;
        steerConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        steerConfig.MotorOutput.Inverted =
                config_.module_type.steerInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> steer_talonfx_.getConfigurator().apply(steerConfig, 0.25));

        // Create drive status signals
        drive_position_sig_ = drive_talonfx_.getPosition();
        drive_velocity_sig_ = drive_talonfx_.getVelocity();
        drive_applied_volts_sig_ = drive_talonfx_.getMotorVoltage();
        drive_current_sig_ = drive_talonfx_.getStatorCurrent();

        // Create steer status signals
        steer_absolute_position_sig_ = steer_talonfx_.getPosition();
        steer_velocity_sig_ = steer_talonfx_.getVelocity();
        steer_applied_volts_sig_ = steer_talonfx_.getMotorVoltage();
        steer_current_sig_ = steer_talonfx_.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
                new CANBus(config_.drive_motor_config.canbus_name).isNetworkFD() ? 250.0 : 100.0,
                steer_absolute_position_sig_,
                drive_position_sig_);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                drive_velocity_sig_,
                drive_applied_volts_sig_,
                drive_current_sig_,
                steer_velocity_sig_,
                steer_applied_volts_sig_,
                steer_current_sig_);
        ParentDevice.optimizeBusUtilizationForAll(drive_talonfx_, steer_talonfx_);

        if (!IS_SIM) {
            PhoenixOdometryThread.getInstance()
                    .registerModule(
                            module_index_, steer_absolute_position_sig_, drive_position_sig_);
        } else {
            simulation.useDriveMotorController(
                    new PhoenixUtil.TalonFXMotorControllerSim(drive_talonfx_));
            simulation.useSteerMotorController(
                    new PhoenixUtil.TalonFXMotorControllerSim(steer_talonfx_));
        }
    }

    @Override
    public void readInputs(double timestamp) {
        // Refresh all signals
        var driveStatus =
                BaseStatusSignal.refreshAll(
                        drive_position_sig_,
                        drive_velocity_sig_,
                        drive_applied_volts_sig_,
                        drive_current_sig_);
        var steerStatus =
                BaseStatusSignal.refreshAll(
                        steer_velocity_sig_, steer_applied_volts_sig_, steer_current_sig_);
        var steerEncoderStatus = BaseStatusSignal.refreshAll(steer_absolute_position_sig_);

        // Update drive inputs
        drive_position_rad_ = Units.rotationsToRadians(drive_position_sig_.getValueAsDouble());
        drive_velocity_rad_per_sec_ =
                Units.rotationsToRadians(drive_velocity_sig_.getValueAsDouble());
        drive_applied_volts_ = drive_applied_volts_sig_.getValueAsDouble();
        drive_current_amps_ = drive_current_sig_.getValueAsDouble();

        // Update steer inputs
        steer_absolute_position_ =
                Rotation2d.fromRotations(steer_absolute_position_sig_.getValueAsDouble());
        steer_velocity_rad_per_sec_ =
                Units.rotationsToRadians(steer_velocity_sig_.getValueAsDouble());
        steer_applied_volts_ = steer_applied_volts_sig_.getValueAsDouble();
        steer_current_amps_ = steer_current_sig_.getValueAsDouble();

        // Update odometry inputs
        if (IS_SIM) {
            double[] odometry_timestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
            double[] odometry_drive_positions_rad =
                    Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
                            .mapToDouble(angle -> angle.in(Radians))
                            .toArray();
            Rotation2d[] odometry_steer_positions = simulation.getCachedSteerAbsolutePositions();

            PhoenixOdometryThread.getInstance()
                    .enqueueModuleSamples(
                            module_index_,
                            odometry_timestamps,
                            odometry_steer_positions,
                            odometry_drive_positions_rad);
        }

        // Update alerts
        drive_disconnected_alert_.set(!drive_conn_deb_.calculate(driveStatus.isOK()));
        steer_disconnected_alert_.set(!steer_conn_deb_.calculate(steerStatus.isOK()));
        steer_encoder_disconnected_alert_.set(
                !steer_encoder_conn_deb_.calculate(steerEncoderStatus.isOK()));
    }

    @Override
    public void setDriveOpenLoop(double output) {
        ControlRequest req;
        if (config_.enable_foc) {
            req = torque_current_req_.withOutput(output);
        } else {
            req = voltage_req_.withOutput(output);
        }
        drive_talonfx_.setControl(req);
    }

    @Override
    public void setSteerOpenLoop(double output) {
        ControlRequest req;
        if (config_.enable_foc) {
            req = voltage_req_.withOutput(output);
        } else {
            req = torque_current_req_.withOutput(output);
        }
        steer_talonfx_.setControl(req);
    }

    @Override
    public void setDriveVelocity(double wheelVelocityRadPerSec) {
        double motorVelocityRotPerSec =
                Units.radiansToRotations(wheelVelocityRadPerSec) * config_.module_type.driveRatio;
        ControlRequest req;
        if (config_.enable_foc) {
            req = velocity_torque_current_req_.withVelocity(motorVelocityRotPerSec);
        } else {
            req = velocity_voltage_req_.withVelocity(motorVelocityRotPerSec);
        }
        drive_talonfx_.setControl(req);
    }

    @Override
    public void setSteerPosition(Rotation2d rotation) {
        ControlRequest req;
        if (config_.enable_foc) {
            req = position_torque_current_req_.withPosition(rotation.getRotations());
        } else {
            req = position_voltage_req_.withPosition(rotation.getRotations());
        }
        steer_talonfx_.setControl(req);
    }

    @Override
    public void setDriveGains(int slot, SlotConfigs gains) {
        if (slot == 0) {
            drive_talonfx_.getConfigurator().apply(Slot0Configs.from(gains));
        } else if (slot == 1) {
            drive_talonfx_.getConfigurator().apply(Slot1Configs.from(gains));
        } else {
            throw new IllegalArgumentException("Slot must be 0 or 1 for drive motor");
        }
    }

    @Override
    public void setSteerGains(SlotConfigs gains) {
        steer_talonfx_.getConfigurator().apply(Slot0Configs.from(gains));
    }

    @Override
    public void setModuleOffset() {
        MWPreferences.getInstance()
                .setPreference(
                        "Encoder" + encoder.getChannel() + "Offset",
                        steer_absolute_position_.getRotations());
        steer_talonfx_.setPosition(0.0);
    }

    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        steer_talonfx_.setNeutralMode(mode);
        drive_talonfx_.setNeutralMode(mode);
        DogLog.log(getLoggingKey() + "SetNeutralMode", mode);
    }

    @Override
    public void writeOutputs(double timestamp) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'writeOutputs'");
    }

    @Override
    public void logData() {
        DogLog.log(getLoggingKey() + "Drive/PositionRad", drive_position_rad_);
        DogLog.log(getLoggingKey() + "Drive/VelocityRadPerSec", drive_velocity_rad_per_sec_);
        DogLog.log(getLoggingKey() + "Drive/AppliedVolts", drive_applied_volts_);
        DogLog.log(getLoggingKey() + "Drive/CurrentAmps", drive_current_amps_);
        DogLog.log(getLoggingKey() + "Steer/AbsolutePosition", steer_absolute_position_);
        DogLog.log(getLoggingKey() + "Steer/VelocityRadPerSec", steer_velocity_rad_per_sec_);
        DogLog.log(getLoggingKey() + "Steer/AppliedVolts", steer_applied_volts_);
        DogLog.log(getLoggingKey() + "Steer/CurrentAmps", steer_current_amps_);
    }
}
