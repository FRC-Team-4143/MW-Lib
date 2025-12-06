package frc.mw_lib.swerve_lib.module;

import static edu.wpi.first.units.Units.Radians;
import static frc.mw_lib.util.PhoenixUtil.tryUntilOk;

import java.util.Arrays;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.ejml.simple.UnsupportedOperation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.mw_lib.mechanisms.MechBase;
import frc.mw_lib.swerve_lib.PhoenixOdometryThread;
import frc.mw_lib.util.PhoenixUtil;

public class ModuleTalonFX extends Module {
  protected final TalonFX drive_talonfx_;
  protected final TalonFX turn_talonfx_;

  // pick one of any of these encoders based on your configuration
  protected final CANcoder cancoder;
  protected final AnalogEncoder encoder;

  // Non FOC control requests
  protected final VoltageOut voltage_req_ = new VoltageOut(0);
  protected final PositionVoltage position_voltage_req_ = new PositionVoltage(0.0);
  protected final VelocityVoltage velocity_voltage_req_ = new VelocityVoltage(0.0);

  // Torque-current control requests
  protected final TorqueCurrentFOC torque_current_req_ = new TorqueCurrentFOC(0);
  protected final PositionTorqueCurrentFOC position_torque_current_req_ = new PositionTorqueCurrentFOC(0.0);
  protected final VelocityTorqueCurrentFOC velocity_torque_current_req_ = new VelocityTorqueCurrentFOC(0.0);

  // Inputs from drive motor
  protected final StatusSignal<Angle> drive_position_sig_;
  protected final StatusSignal<AngularVelocity> drive_velocity_sig_;
  protected final StatusSignal<Voltage> drive_applied_volts_sig_;
  protected final StatusSignal<Current> drive_current_sig_;

  // Inputs from turn motor
  protected final StatusSignal<Angle> turn_absolute_position_sig_;
  protected final StatusSignal<AngularVelocity> turn_velocity_sig_;
  protected final StatusSignal<Voltage> turn_applied_volts_sig_;
  protected final StatusSignal<Current> turn_current_sig_;

  public ModuleTalonFX(SwerveModuleConfig config, SwerveModuleSimulation simulation) {
    super(config, simulation);

    drive_talonfx_ = new TalonFX(config_.drive_motor_config.can_id, config_.drive_motor_config.canbus_name);
    turn_talonfx_ = new TalonFX(config_.steer_motor_config.can_id, config_.steer_motor_config.canbus_name);

    if (config_.encoder_type != SwerveModuleConfig.EncoderType.ANALOG_ENCODER) {
      cancoder = new CANcoder(config_.encoder_id, config_.drive_motor_config.canbus_name);
      encoder = null;
    } else {
      cancoder = null;
      encoder = new AnalogEncoder(config_.encoder_id);
    }

    // Configure drive motor
    tryUntilOk(5, () -> drive_talonfx_.getConfigurator().apply(config_.drive_motor_config.config, 0.25));
    tryUntilOk(5, () -> drive_talonfx_.setPosition(0.0, 0.25));

    // Configure turn motor
    var turnConfig = config_.steer_motor_config.config;
    turnConfig.Feedback.RotorToSensorRatio = config_.module_type.steerRatio;
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // config the encoder offset for the steer motor
    if (config_.encoder_type == SwerveModuleConfig.EncoderType.ANALOG_ENCODER) {
      // Use analog encoder
      // TODO CJT implement for analog encoder
      throw new UnsupportedOperation("Analog encoder not yet supported in ModuleTalonFX");
    } else if (config_.encoder_type == SwerveModuleConfig.EncoderType.CTRE_CAN_CODER) {
      // Use remote CANCoder
      // TODO CJT implement for cancoder
      throw new UnsupportedOperation("CANCoder not yet supported in ModuleTalonFX");
    } else {
      throw new IllegalArgumentException("Unsupported encoder type for ModuleTalonFX");
    }
    
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / config_.module_type.steerRatio;
    turnConfig.MotionMagic.MotionMagicAcceleration = turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * config_.module_type.steerRatio;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted = config_.module_type.steerInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turn_talonfx_.getConfigurator().apply(turnConfig, 0.25));

    // Create drive status signals
    drive_position_sig_ = drive_talonfx_.getPosition();
    drive_velocity_sig_ = drive_talonfx_.getVelocity();
    drive_applied_volts_sig_ = drive_talonfx_.getMotorVoltage();
    drive_current_sig_ = drive_talonfx_.getStatorCurrent();

    // Create turn status signals
    turn_absolute_position_sig_ = turn_talonfx_.getPosition();
    turn_velocity_sig_ = turn_talonfx_.getVelocity();
    turn_applied_volts_sig_ = turn_talonfx_.getMotorVoltage();
    turn_current_sig_ = turn_talonfx_.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        new CANBus(config_.drive_motor_config.canbus_name).isNetworkFD() ? 250.0 : 100.0, turn_absolute_position_sig_,
        drive_position_sig_);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        drive_velocity_sig_,
        drive_applied_volts_sig_,
        drive_current_sig_,
        turn_velocity_sig_,
        turn_applied_volts_sig_,
        turn_current_sig_);
    ParentDevice.optimizeBusUtilizationForAll(drive_talonfx_, turn_talonfx_);

    if (!IS_SIM) {
      this.timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
      this.drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drive_position_sig_);
      this.turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turn_absolute_position_sig_);
    } else {
      simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(drive_talonfx_));
      simulation.useSteerMotorController(new PhoenixUtil.TalonFXMotorControllerSim(turn_talonfx_));
    }
  }

  @Override
  public void readInputs(double timestamp) {
    // Refresh all signals
    var driveStatus = BaseStatusSignal.refreshAll(drive_position_sig_, drive_velocity_sig_, drive_applied_volts_sig_,
        drive_current_sig_);
    var turnStatus = BaseStatusSignal.refreshAll(turn_velocity_sig_, turn_applied_volts_sig_, turn_current_sig_);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turn_absolute_position_sig_);

    // Update drive inputs
    drive_position_rad_ = Units.rotationsToRadians(drive_position_sig_.getValueAsDouble());
    drive_velocity_rad_per_sec_ = Units.rotationsToRadians(drive_velocity_sig_.getValueAsDouble());
    drive_applied_volts_ = drive_applied_volts_sig_.getValueAsDouble();
    drive_current_amps_ = drive_current_sig_.getValueAsDouble();

    // Update turn inputs
    turn_absolute_position_ = Rotation2d.fromRotations(turn_absolute_position_sig_.getValueAsDouble());
    turn_velocity_rad_per_sec_ = Units.rotationsToRadians(turn_velocity_sig_.getValueAsDouble());
    turn_applied_volts_ = turn_applied_volts_sig_.getValueAsDouble();
    turn_current_amps_ = turn_current_sig_.getValueAsDouble();

    // Update odometry inputs
    if (!IS_SIM) {
      odometry_timestamps_ = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
      odometry_drive_positions_rad_ = drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
      odometry_turn_positions_ = turnPositionQueue.stream().map(Rotation2d::fromRotations)
          .toArray(Rotation2d[]::new);
      timestampQueue.clear();
      drivePositionQueue.clear();
      turnPositionQueue.clear();
    } else {
      odometry_timestamps_ = PhoenixUtil.getSimulationOdometryTimeStamps();
      odometry_drive_positions_rad_ = Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
          .mapToDouble(angle -> angle.in(Radians))
          .toArray();
      odometry_turn_positions_ = simulation.getCachedSteerAbsolutePositions();
    }

    // Update alerts
    drive_disconnected_alert_.set(!drive_conn_deb_.calculate(driveStatus.isOK()));
    turn_disconnected_alert_.set(!turn_conn_deb_.calculate(turnStatus.isOK()));
    turn_encoder_disconnected_alert_.set(!turn_encoder_conn_deb_.calculate(turnEncoderStatus.isOK()));
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
  public void setTurnOpenLoop(double output) {
    ControlRequest req;
    if (config_.enable_foc) {
      req = voltage_req_.withOutput(output);
    } else {
      req = torque_current_req_.withOutput(output);
    }
    turn_talonfx_.setControl(req);
  }

  @Override
  public void setDriveVelocity(double wheelVelocityRadPerSec) {
    double motorVelocityRotPerSec = Units.radiansToRotations(wheelVelocityRadPerSec) * config_.module_type.driveRatio;
    ControlRequest req;
    if (config_.enable_foc) {
      req = velocity_torque_current_req_.withVelocity(motorVelocityRotPerSec);
    } else {
      req = velocity_voltage_req_.withVelocity(motorVelocityRotPerSec);
    }
    drive_talonfx_.setControl(req);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    ControlRequest req;
    if (config_.enable_foc) {
      req = position_torque_current_req_.withPosition(rotation.getRotations());
    } else {
      req = position_voltage_req_.withPosition(rotation.getRotations());
    }
    turn_talonfx_.setControl(req);
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
    DogLog.log(getLoggingKey() + "Turn/AbsolutePosition", turn_absolute_position_);
    DogLog.log(getLoggingKey() + "Turn/VelocityRadPerSec", turn_velocity_rad_per_sec_);
    DogLog.log(getLoggingKey() + "Turn/AppliedVolts", turn_applied_volts_);
    DogLog.log(getLoggingKey() + "Turn/CurrentAmps", turn_current_amps_);
  }
}
