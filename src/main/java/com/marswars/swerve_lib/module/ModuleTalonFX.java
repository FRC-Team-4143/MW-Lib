package com.marswars.swerve_lib.module;

import static com.marswars.util.PhoenixUtil.tryUntilOk;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import com.marswars.swerve_lib.PhoenixOdometryThread;
import com.marswars.util.MWPreferences;
import org.ejml.simple.UnsupportedOperation;

/**
 * TalonFX-based swerve module implementation with optional simulation support.
 */
public class ModuleTalonFX extends Module {
    private final boolean IS_SIM = RobotBase.isSimulation();
    
    // Motor models for simulation
    private static final DCMotor DRIVE_MOTOR_MODEL = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor STEER_MOTOR_MODEL = DCMotor.getKrakenX44Foc(1);
    private static final double SIM_PERIOD_SECS = 0.02;
    
    protected final TalonFX drive_talonfx_;
    protected final TalonFX steer_talonfx_;

    // pick one of any of these encoders based on your configuration
    protected final CANcoder cancoder;
    protected final AnalogEncoder encoder;

    // Non FOC control requests
    protected final VoltageOut voltage_req_ = new VoltageOut(0);
    protected final PositionVoltage position_voltage_req_ = new PositionVoltage(0.0).withSlot(0);
    protected final VelocityVoltage velocity_voltage_req_ = new VelocityVoltage(0.0).withSlot(1);

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

    // Analog encoder input (if used)
    protected double encoder_value_abs_;
    
    // Encoder correlation detection (check if encoder changes when motor moves)
    private double prev_encoder_value_ = 0.0;
    private double prev_steer_velocity_ = 0.0;
    private static final double ENCODER_CHANGE_THRESHOLD = 0.001; // Minimum encoder change to detect movement
    private static final double MOTOR_VELOCITY_THRESHOLD = 0.1; // rad/s - motor is "moving" above this
    private int correlation_check_count_ = 0;
    private int correlation_fail_count_ = 0;
    private static final int CORRELATION_CHECK_SAMPLES = 10; // Check over 10 samples (~0.2s at 50Hz)
    
    // Simulation objects (only used when IS_SIM is true)
    private DCMotorSim drive_sim_;
    private DCMotorSim steer_sim_;
    private PIDController drive_controller_;
    private PIDController steer_controller_;
    private boolean drive_closed_loop_ = false;
    private boolean steer_closed_loop_ = false;
    private double drive_ff_volts_ = 0.0;
    private double sim_drive_applied_volts_ = 0.0;
    private double sim_steer_applied_volts_ = 0.0;

    /**
     * Creates a new TalonFX swerve module.
     *
     * @param logging_prefix Logging prefix for telemetry
     * @param index Module index (0-3)
     * @param config Module configuration
     */
    public ModuleTalonFX(
        String logging_prefix,
        int index,
        SwerveModuleConfig config) {
        super(logging_prefix, index, config);

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
            // Allow time for analog encoder to stabilize before reading
            Timer.delay(0.01);
        }

        // Configure drive motor
        config_.drive_motor_config.config.Feedback.SensorToMechanismRatio = config_.module_type.driveRatio;
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
            String offset_name = "Encoder" + module_index_ + "Offset";
            Rotation2d encoder_offset =
                    Rotation2d.fromRotations(
                            MWPreferences.getInstance()
                                    .getPreferenceDouble(offset_name, 0.0));
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

        PhoenixOdometryThread.getInstance()
                .registerModule(
                        module_index_, steer_absolute_position_sig_, drive_position_sig_);
        
        // Initialize simulation objects if in simulation mode
        if (IS_SIM) {
            initializeSimulation();
        }

        DogLog.log(getLoggingKey() + "ModuleType", config_.module_type.name);
    }
    
    private void initializeSimulation() {
        // Create drive motor simulation
        drive_sim_ =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                DRIVE_MOTOR_MODEL, 0.025, config_.module_type.driveRatio),
                        DRIVE_MOTOR_MODEL);
        
        // Create steer motor simulation
        steer_sim_ =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                STEER_MOTOR_MODEL, 0.004, config_.module_type.steerRatio),
                        STEER_MOTOR_MODEL);
        
        // Create controllers for closed-loop simulation
        drive_controller_ = new PIDController(0.1, 0.0, 0.0, SIM_PERIOD_SECS);
        steer_controller_ = new PIDController(10.0, 0.0, 0.0, SIM_PERIOD_SECS);
        
        // Enable continuous input for steer controller
        steer_controller_.enableContinuousInput(-Math.PI, Math.PI);
        
        // Set initial steer position to match encoder offset
        steer_sim_.setState(config_.encoder_offset_rad, 0.0);
    }

    /** {@inheritDoc} */
    @Override
    public void readInputs(double timestamp) {
        if (IS_SIM) {
            readInputsSimulation();
        } else {
            readInputsReal();
        }
    }
    
    private void readInputsSimulation() {
        // Run closed-loop controllers
        if (drive_closed_loop_) {
            sim_drive_applied_volts_ =
                    drive_ff_volts_
                            + drive_controller_.calculate(drive_sim_.getAngularVelocityRadPerSec());
        } else {
            drive_controller_.reset();
        }
        
        if (steer_closed_loop_) {
            sim_steer_applied_volts_ = steer_controller_.calculate(steer_sim_.getAngularPositionRad());
        } else {
            steer_controller_.reset();
        }
        
        // Update simulation state
        drive_sim_.setInputVoltage(MathUtil.clamp(sim_drive_applied_volts_, -12.0, 12.0));
        steer_sim_.setInputVoltage(MathUtil.clamp(sim_steer_applied_volts_, -12.0, 12.0));
        drive_sim_.update(SIM_PERIOD_SECS);
        steer_sim_.update(SIM_PERIOD_SECS);
        
        // Update drive inputs from simulation
        drive_position_rad_ = drive_sim_.getAngularPositionRad();
        drive_velocity_rad_per_sec_ = drive_sim_.getAngularVelocityRadPerSec();
        drive_applied_volts_ = sim_drive_applied_volts_;
        drive_current_amps_ = Math.abs(drive_sim_.getCurrentDrawAmps());
        
        // Update steer inputs from simulation
        steer_absolute_position_ = new Rotation2d(steer_sim_.getAngularPositionRad());
        steer_velocity_rad_per_sec_ = steer_sim_.getAngularVelocityRadPerSec();
        steer_applied_volts_ = sim_steer_applied_volts_;
        steer_current_amps_ = Math.abs(steer_sim_.getCurrentDrawAmps());
        
        // Simulation is always "connected"
        drive_disconnected_alert_.set(false);
        steer_disconnected_alert_.set(false);
        module_encoder_alert_.set(false);
        
        // Enqueue odometry sample for simulation
        // Create single-sample arrays with current timestamp and positions
        double currentTime = Timer.getFPGATimestamp();
        PhoenixOdometryThread.getInstance().enqueueModuleSamples(
                module_index_,
                new double[] {currentTime},
                new Rotation2d[] {steer_absolute_position_},
                new double[] {drive_position_rad_});
    }
    
    private void readInputsReal() {
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

        // Update alerts
        drive_disconnected_alert_.set(!drive_conn_deb_.calculate(driveStatus.isOK()));
        steer_disconnected_alert_.set(!steer_conn_deb_.calculate(steerStatus.isOK()));

        // Update encoder value if using analog encoder
        if (encoder != null) {
            encoder_value_abs_ = encoder.get();
            
            // Detect disconnection by checking if encoder changes when motor moves
            double encoder_change = Math.abs(encoder_value_abs_ - prev_encoder_value_);
            double steer_velocity_abs = Math.abs(steer_velocity_rad_per_sec_);
            
            // Only check correlation when motor is moving significantly
            if (steer_velocity_abs > MOTOR_VELOCITY_THRESHOLD) {
                correlation_check_count_++;
                
                // If motor is moving but encoder isn't changing, that's a fail
                if (encoder_change < ENCODER_CHANGE_THRESHOLD) {
                    correlation_fail_count_++;
                }
                
                // After collecting enough samples, check the correlation
                if (correlation_check_count_ >= CORRELATION_CHECK_SAMPLES) {
                    // If encoder failed to move in most samples, it's likely disconnected
                    boolean encoder_is_stuck = correlation_fail_count_ > (CORRELATION_CHECK_SAMPLES / 2);
                    module_encoder_alert_.set(encoder_is_stuck);
                    
                    // Reset counters for next check
                    correlation_check_count_ = 0;
                    correlation_fail_count_ = 0;
                }
            }
            
            // Update previous values for next iteration
            prev_encoder_value_ = encoder_value_abs_;
            prev_steer_velocity_ = steer_velocity_rad_per_sec_;
        }
    }

    /** {@inheritDoc} */
    @Override
    public void setDriveOpenLoop(double output) {
        if (IS_SIM) {
            drive_closed_loop_ = false;
            sim_drive_applied_volts_ = output;
            drive_ff_volts_ = 0.0;
        } else {
            ControlRequest req;
            if (config_.enable_foc) {
                req = torque_current_req_.withOutput(output);
            } else {
                req = voltage_req_.withOutput(output);
            }
            drive_talonfx_.setControl(req);
        }
    }

    /** {@inheritDoc} */
    @Override
    public void setSteerOpenLoop(double output) {
        if (IS_SIM) {
            steer_closed_loop_ = false;
            sim_steer_applied_volts_ = output;
        } else {
            ControlRequest req;
            if (config_.enable_foc) {
                req = torque_current_req_.withOutput(output);
            } else {
                req = voltage_req_.withOutput(output);
            }
            steer_talonfx_.setControl(req);
        }
    }

    /** {@inheritDoc} */
    @Override
    public void setDriveVelocity(double wheelVelocityRadPerSec) {
        if (IS_SIM) {
            drive_closed_loop_ = true;
            drive_controller_.setSetpoint(wheelVelocityRadPerSec);
            // Calculate feedforward
            double kV = 12.0 / (config_.speed_at_12_volts / config_.wheel_radius_m);
            drive_ff_volts_ = kV * wheelVelocityRadPerSec;
        } else {
            double motorVelocityRotPerSec = Units.radiansToRotations(wheelVelocityRadPerSec);
            ControlRequest req;
            if (config_.enable_foc) {
                req = velocity_torque_current_req_.withVelocity(motorVelocityRotPerSec);
            } else {
                req = velocity_voltage_req_.withVelocity(motorVelocityRotPerSec);
            }
            drive_talonfx_.setControl(req);
        }
    }

    /** {@inheritDoc} */
    @Override
    public void setSteerPosition(Rotation2d rotation) {
        if (IS_SIM) {
            steer_closed_loop_ = true;
            steer_controller_.setSetpoint(rotation.getRadians());
        } else {
            ControlRequest req;
            if (config_.enable_foc) {
                req = position_torque_current_req_.withPosition(rotation.getRotations());
            } else {
                req = position_voltage_req_.withPosition(rotation.getRotations());
            }
            steer_talonfx_.setControl(req);
        }
    }

    /** {@inheritDoc} */
    @Override
    public void setDriveGains(int slot, SlotConfigs gains) {
        if (IS_SIM) {
            drive_controller_.setP(gains.kP);
            drive_controller_.setI(gains.kI);
            drive_controller_.setD(gains.kD);
        } else {
            if (slot == 0) {
                drive_talonfx_.getConfigurator().apply(Slot0Configs.from(gains));
            } else if (slot == 1) {
                drive_talonfx_.getConfigurator().apply(Slot1Configs.from(gains));
            } else {
                throw new IllegalArgumentException("Slot must be 0 or 1 for drive motor");
            }
        }
    }

    /** {@inheritDoc} */
    @Override
    public void setSteerGains(SlotConfigs gains) {
        if (IS_SIM) {
            steer_controller_.setP(gains.kP);
            steer_controller_.setI(gains.kI);
            steer_controller_.setD(gains.kD);
        } else {
            steer_talonfx_.getConfigurator().apply(Slot0Configs.from(gains));
        }
    }

    /** {@inheritDoc} */
    @Override
    public void setModuleOffset() {
        if (IS_SIM) {
            // In simulation, reset the steer position to zero
            steer_sim_.setState(0.0, steer_sim_.getAngularVelocityRadPerSec());
        } else {
            MWPreferences.getInstance()
                    .setPreference(
                            "Encoder" + module_index_ + "Offset",
                            steer_absolute_position_.getRotations());
            steer_talonfx_.setPosition(0.0);
        }
    }

    /** {@inheritDoc} */
    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        neutral_mode_ = mode;
        if (!IS_SIM) {
            steer_talonfx_.setNeutralMode(mode);
            drive_talonfx_.setNeutralMode(mode);
        }
        DogLog.log(getLoggingKey() + "NeutralMode", mode);
    }

    /** {@inheritDoc} */
    @Override
    public void writeOutputs(double timestamp) {
        // In simulation, stop motors when disabled
        if (IS_SIM && DriverStation.isDisabled()) {
            drive_closed_loop_ = false;
            steer_closed_loop_ = false;
            sim_drive_applied_volts_ = 0.0;
            sim_steer_applied_volts_ = 0.0;
        }
        // For real hardware, control requests are sent immediately in the set methods
    }

    /** {@inheritDoc} */
    @Override
    public void logData() {
        DogLog.log(getLoggingKey() + "Drive/PositionRad", drive_position_rad_, Radians);
        DogLog.log(getLoggingKey() + "Drive/VelocityRadPerSec", drive_velocity_rad_per_sec_, RadiansPerSecond);
        DogLog.log(getLoggingKey() + "Drive/AppliedVolts", drive_applied_volts_, Volts);
        DogLog.log(getLoggingKey() + "Drive/CurrentAmps", drive_current_amps_, Amps);
        DogLog.log(getLoggingKey() + "Steer/AbsolutePosition", steer_absolute_position_.getRadians(), Radians);
        DogLog.log(getLoggingKey() + "Steer/VelocityRadPerSec", steer_velocity_rad_per_sec_, RadiansPerSecond);
        DogLog.log(getLoggingKey() + "Steer/AppliedVolts", steer_applied_volts_, Volts);
        DogLog.log(getLoggingKey() + "Steer/CurrentAmps", steer_current_amps_, Amps);
        DogLog.log(getLoggingKey() + "NeutralMode", neutral_mode_);
        DogLog.log(getLoggingKey() + "Encoder/AbsoluteValue", encoder_value_abs_, Rotation);
    }
}
