package com.marswars.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.traits.CommonTalon;

import com.marswars.logging.MwLog;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.util.TunablePid;
import java.util.List;

/**
 * TalonFX-based mechanism implementation for rollers with position, velocity, and duty cycle control.
 */
public class RollerMech extends MechBase {

    /** Control modes for the roller mechanism */
    protected enum ControlMode {
        MOTION_PROFILE_POSITION,
        POSITION,
        MOTION_PROFILE_VELOCITY,
        VELOCITY,
        DUTY_CYCLE,
        CURRENT
    }

    private ControlMode control_mode_ = ControlMode.DUTY_CYCLE;

    // Temperature threshold for alerts (Celsius)
    private static final double MOTOR_TEMP_THRESHOLD_C = 65.0;

    // Always assume that we have the leader motor in index 0
    private final CommonTalon motors_[];
    private final PositionVoltage position_request_;
    protected final MotionMagicVoltage motion_magic_position_request_;
    private final VelocityVoltage velocity_request_;
    protected final MotionMagicVelocityVoltage motion_magic_velocity_request_;
    private final DutyCycleOut duty_cycle_request_;
    private final DutyCycleOut current_request_;
    protected final BaseStatusSignal[] signals_;
    private final PIDController current_pid_;
    private final LinearFilter current_filter_;

    // Alerts for motor monitoring
    protected final Alert[] motor_disconnected_alerts_;
    protected final Alert[] motor_temp_alerts_;
    protected final Debouncer[] motor_conn_debouncers_;

    // AdvantageKit inputs (sensor reads only — replayed from log; single source of truth)
    protected final MechInputsAutoLogged inputs_ = new MechInputsAutoLogged();

    // command targets (outputs — computed each loop, NOT replayed)
    protected double position_target_ = 0;
    protected double velocity_target_ = 0;
    protected double duty_cycle_target_ = 0;
    protected double current_target_ = 0;
    protected double current_target_FF_ = 0;
    protected double filtered_torque_current_ = 0;


    // System parameters
    private final double gear_ratio_;
    private final double roller_inertia_;

    // Simulation
    private final DCMotor motor_type_;
    private final DCMotorSim roller_sim_;
    private double sim_load_torque_nm_ = 0.0; // Load torque at roller shaft for simulation

    /**
     * Constructs a new RollerMech with a default inertia value.
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs Configuration for the roller motor
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     */
    public RollerMech(String logging_prefix, List<MotorConfig> motor_configs, double gear_ratio) {
        this(logging_prefix, null, motor_configs, gear_ratio, 0.00001);
    }

    /**
     * Constructs a new RollerMech
     *
     * @param logging_prefix String prefix for logging
     * @param mech_name Name of the mechanism
     * @param motor_configs Configuration for the roller motor
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     */
    public RollerMech(String logging_prefix, String mech_name, List<MotorConfig> motor_configs, double gear_ratio) {
        this(logging_prefix, mech_name, motor_configs, gear_ratio, 0.00001);
    }

    /**
     * Constructs a new RollerMech
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs Configuration for the roller motor
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param roller_inertia Inertia of the roller in kg*m^2 (Simulation only)
     */
    public RollerMech(String logging_prefix, List<MotorConfig> motor_configs, double gear_ratio, double roller_inertia) {
        this(logging_prefix, null, motor_configs, gear_ratio, roller_inertia);
    }

    /**
     * Constructs a new RollerMech
     *
     * @param logging_prefix String prefix for logging
     * @param mech_name Name of the mechanism
     * @param motor_configs Configuration for the roller motor
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param roller_inertia Inertia of the roller in kg*m^2 (Simulation only)
     */
    public RollerMech(String logging_prefix, String mech_name, List<MotorConfig> motor_configs, double gear_ratio, double roller_inertia) {
        super(logging_prefix, mech_name);

        gear_ratio_ = gear_ratio;
        roller_inertia_ = roller_inertia;

        // MW-Lib convention: gear_ratio is motor/mechanism
        // Phoenix convention: SensorToMechanismRatio = sensor/mechanism = motor/mechanism
        // These are the same, so we can use gear_ratio directly
        double sensor_to_mech_ratio = gear_ratio_;
        
        MechBase.ConstructedMotors configured_motors = 
                configMotors(motor_configs, sensor_to_mech_ratio);

        // Store system parameters
        position_request_ = new PositionVoltage(0).withSlot(0);
        motion_magic_position_request_ = new MotionMagicVoltage(0).withSlot(0);
        velocity_request_ = new VelocityVoltage(0).withSlot(1);
        motion_magic_velocity_request_ = new MotionMagicVelocityVoltage(0).withSlot(1);
        duty_cycle_request_ = new DutyCycleOut(0);
        current_request_ = new DutyCycleOut(0);

        // convert the list to an array for easy access
        motors_ = configured_motors.motors;
        signals_ = configured_motors.signals;

        // size array fields in the inputs struct to match motor count
        inputs_.appliedVoltage    = new double[motors_.length];
        inputs_.supplyCurrentDraw = new double[motors_.length];
        inputs_.statorCurrentDraw = new double[motors_.length];
        inputs_.torqueCurrentDraw = new double[motors_.length];
        inputs_.motorTempC        = new double[motors_.length];
        inputs_.busVoltage        = new double[motors_.length];

        // Initialize alerts and debouncers for each motor
        motor_disconnected_alerts_ = new Alert[motors_.length];
        motor_temp_alerts_ = new Alert[motors_.length];
        motor_conn_debouncers_ = new Debouncer[motors_.length];
        for (int i = 0; i < motors_.length; i++) {
            motor_disconnected_alerts_[i] = new Alert(
                    "Disconnected motor " + i + " in " + getLoggingKey(),
                    AlertType.kError);
            motor_temp_alerts_[i] = new Alert(
                    "High temperature on motor " + i + " in " + getLoggingKey(),
                    AlertType.kWarning);
            motor_conn_debouncers_[i] = new Debouncer(0.5);
        }

        //////////////////////////
        /// SIMULATION SETUP ///
        //////////////////////////

        if (motor_configs.get(0).motor_type == TalonMotorType.X60) {
            motor_type_ = DCMotor.getKrakenX60(motor_configs.size());
        } else if (motor_configs.get(0).motor_type == TalonMotorType.X44) {
            motor_type_ = DCMotor.getKrakenX44(motor_configs.size());
        } else if (motor_configs.get(0).motor_type == TalonMotorType.FALCON500) {
            motor_type_ = DCMotor.getFalcon500(motor_configs.size());
        } else if (motor_configs.get(0).motor_type == TalonMotorType.MINION) {
            motor_type_ = DCMotor.getKrakenX44Foc(motor_configs.size());
        } else if (motor_configs.get(0).motor_type == TalonMotorType.NEO_550) {
            motor_type_ = DCMotor.getNeo550(motor_configs.size());
        } else {
            throw new IllegalArgumentException("Unsupported motor type: " + motor_configs.get(0).motor_type);
        }

        roller_sim_ =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                motor_type_, roller_inertia_, gear_ratio_),
                        motor_type_);

        // Setup tunable PIDs
        SlotConfigs slot0Config;
        SlotConfigs slot1Config;
        SlotConfigs slot2Configs;
        if (motor_configs.get(0).isFXS()) {
            TalonFXSConfiguration fxsConfig = motor_configs.get(0).getAsFXSConfig();
            slot0Config = SlotConfigs.from(fxsConfig.Slot0);
            slot1Config = SlotConfigs.from(fxsConfig.Slot1);
            slot2Configs = SlotConfigs.from(fxsConfig.Slot2);
        } else {
            TalonFXConfiguration fxConfig = motor_configs.get(0).getAsFXConfig();
            slot0Config = SlotConfigs.from(fxConfig.Slot0);
            slot1Config = SlotConfigs.from(fxConfig.Slot1);
            slot2Configs = SlotConfigs.from(fxConfig.Slot2);
        }
        current_pid_ = new PIDController(slot2Configs.kP, slot2Configs.kI, slot2Configs.kD);
        current_filter_ = LinearFilter.singlePoleIIR(0.1, 0.02);

        TunablePid.create(
                getLoggingKey() + "PositionGains",
                this::configPositionSlot,
                slot0Config);
        MwLog.tunable(
                getLoggingKey() + "PositionGains/Setpoint", 0.0, (val) -> setTargetPosition(val));
        TunablePid.create(
                getLoggingKey() + "VelocityGains",
                this::configVelocitySlot,
                slot1Config);
        MwLog.tunable(
                getLoggingKey() + "VelocityGains/Setpoint", 0.0, (val) -> setTargetVelocity(val));
        MwLog.tunable(
                getLoggingKey() + "DutyCycle/Setpoint", 0.0, (val) -> setTargetDutyCycle(val));
        TunablePid.create(getLoggingKey() + "CurrentGains", current_pid_);
        MwLog.tunable(
                getLoggingKey() + "CurrentGains/Setpoint", 0.0, (val) -> setTargetCurrent(val));
    }

    /** {@inheritDoc} */
    @Override
    public void readInputs(double timestamp) {
        if (!MwLog.isReplay()) {
            BaseStatusSignal.refreshAll(signals_);

            inputs_.position = motors_[0].getPosition().getValue().in(Radians);
            inputs_.velocity = motors_[0].getVelocity().getValue().in(RadiansPerSecond);
            for (int i = 0; i < motors_.length; i++) {
                inputs_.appliedVoltage[i]    = motors_[i].getMotorVoltage().getValueAsDouble();
                inputs_.supplyCurrentDraw[i] = motors_[i].getSupplyCurrent().getValue().in(Amps);
                inputs_.statorCurrentDraw[i] = motors_[i].getStatorCurrent().getValue().in(Amps);
                inputs_.torqueCurrentDraw[i] = motors_[i].getTorqueCurrent().getValue().in(Amps);
                inputs_.motorTempC[i]     = motors_[i].getDeviceTemp().getValue().in(Celsius);
                inputs_.busVoltage[i]     = motors_[i].getSupplyVoltage().getValueAsDouble();

                motor_disconnected_alerts_[i].set(
                        !motor_conn_debouncers_[i].calculate(motors_[i].isConnected()));
                motor_temp_alerts_[i].set(inputs_.motorTempC[i] > MOTOR_TEMP_THRESHOLD_C);
            }

            // Simulation physics — guarded here so it doesn't run during replay,
            // which also executes in sim mode.
            if (IS_SIM) {
                for (int i = 0; i < motors_.length; i++) {
                    if (motors_[i] instanceof TalonFX) {
                        ((TalonFX) motors_[i]).getSimState().setSupplyVoltage(12.0);
                    } else if (motors_[i] instanceof TalonFXS) {
                        ((TalonFXS) motors_[i]).getSimState().setSupplyVoltage(12.0);
                    }
                }

                double controller_voltage = 0.0;
                if (motors_[0] instanceof TalonFX) {
                    controller_voltage = ((TalonFX) motors_[0]).getSimState().getMotorVoltage();
                } else if (motors_[0] instanceof TalonFXS) {
                    controller_voltage = ((TalonFXS) motors_[0]).getSimState().getMotorVoltage();
                }

                double motor_load_torque = sim_load_torque_nm_ * gear_ratio_;
                double load_current = motor_load_torque / motor_type_.KtNMPerAmp;
                double effective_voltage = controller_voltage - (load_current * motor_type_.rOhms);

                roller_sim_.setInput(effective_voltage);
                roller_sim_.update(0.020);
                sim_load_torque_nm_ = 0.0;

                double mechanismPositionRad = roller_sim_.getAngularPositionRad();
                double mechanismVelocityRadPerSec = roller_sim_.getAngularVelocityRadPerSec();
                double motorPosition = Radians.of(mechanismPositionRad * gear_ratio_).in(Rotations);
                double motorVelocity = RadiansPerSecond.of(mechanismVelocityRadPerSec * gear_ratio_).in(RotationsPerSecond);

                for (int i = 0; i < motors_.length; i++) {
                    if (motors_[i] instanceof TalonFX) {
                        ((TalonFX) motors_[i]).getSimState().setRawRotorPosition(motorPosition);
                        ((TalonFX) motors_[i]).getSimState().setRotorVelocity(motorVelocity);
                    } else if (motors_[i] instanceof TalonFXS) {
                        ((TalonFXS) motors_[i]).getSimState().setRawRotorPosition(motorPosition);
                        ((TalonFXS) motors_[i]).getSimState().setRotorVelocity(motorVelocity);
                    }
                    motor_disconnected_alerts_[i].set(false);
                    motor_temp_alerts_[i].set(false);
                }
            }
        }
        // Records inputs_ to the log (real/sim) or restores inputs_ from the log (replay).
        Logger.processInputs(getLoggingKey() + "Inputs", inputs_);
    }

    /** {@inheritDoc} */
    @Override
    public void writeOutputs(double timestamp) {
        switch (control_mode_) {
            case MOTION_PROFILE_POSITION:
                motors_[0].setControl(motion_magic_position_request_);
                break;
            case POSITION:
                motors_[0].setControl(position_request_);
                break;
            case MOTION_PROFILE_VELOCITY:
                motors_[0].setControl(motion_magic_velocity_request_);
                break;
            case VELOCITY:
                motors_[0].setControl(velocity_request_);
                break;
            case DUTY_CYCLE:
                motors_[0].setControl(duty_cycle_request_);
                break;
            case CURRENT:
                filtered_torque_current_ = current_filter_.calculate(inputs_.torqueCurrentDraw[0]);
                double duty_cycle_output = current_pid_.calculate(filtered_torque_current_, current_target_);
                current_request_.Output = duty_cycle_output;
                motors_[0].setControl(current_request_);
                break;
            default:
                throw new IllegalStateException("Unexpected control mode: " + control_mode_);
        }
    }

    /** {@inheritDoc} */
    @Override
    public void logData() {
        // commands
        MwLog.log(getLoggingKey() + "control/mode", control_mode_.toString());
        MwLog.log(getLoggingKey() + "control/position/target", position_target_, Radians);
        MwLog.log(getLoggingKey() + "control/position/actual", inputs_.position, Radians);
        MwLog.log(getLoggingKey() + "control/velocity/target", velocity_target_, RadiansPerSecond);
        MwLog.log(getLoggingKey() + "control/velocity/actual", inputs_.velocity, RadiansPerSecond);
        MwLog.log(getLoggingKey() + "control/duty_cycle/target", duty_cycle_target_, Percent);
        MwLog.log(getLoggingKey() + "control/duty_cycle/actual", inputs_.appliedVoltage[0] / 12.0, Percent);
        MwLog.log(getLoggingKey() + "control/current/target", current_target_, Amps);
        MwLog.log(getLoggingKey() + "control/current/actual", inputs_.torqueCurrentDraw[0], Amps);
    }

    /**
     * Configures the position slot with the given config
     *
     * @param config the slot config to apply
     */
    private void configPositionSlot(SlotConfigs config) {
        configSlot(0, config);
    }

    /**
     * Configures the velocity slot with the given config
     *
     * @param config the slot config to apply
     */
    private void configVelocitySlot(SlotConfigs config) {
        configSlot(1, config);
    }

    /**
     * Configures the given slot with the given config
     *
     * @param slot the slot index to configure
     * @param config the slot config to apply
     */
    public void configSlot(int slot, SlotConfigs config) {
        if (slot == 0) {
            if (motors_[0] instanceof TalonFX) {
                ((TalonFX) motors_[0]).getConfigurator().apply(Slot0Configs.from(config));
            } else if (motors_[0] instanceof TalonFXS) {
                ((TalonFXS) motors_[0]).getConfigurator().apply(Slot0Configs.from(config));
            }
        } else if (slot == 1) {
            if (motors_[0] instanceof TalonFX) {
                ((TalonFX) motors_[0]).getConfigurator().apply(Slot1Configs.from(config));
            } else if (motors_[0] instanceof TalonFXS) {
                ((TalonFXS) motors_[0]).getConfigurator().apply(Slot1Configs.from(config));
            }
        } else if (slot == 2) {
            // Slot 2 is used for current control PID, so we don't apply it to the motor controller
            // Instead, we just update our PID controller gains
            current_pid_.setP(config.kP);
            current_pid_.setI(config.kI);
            current_pid_.setD(config.kD);
            current_pid_.reset();
        } else {
            throw new IllegalArgumentException("Slot must be 0, 1, or 2");
        }
    }

    /**
     * Sets the current position of the roller mechanism (for resetting encoders, etc.)
     *
     * @param position_rad the position in radians
     */
    public void setCurrentPosition(double position_rad) {
        motors_[0].setPosition(Units.radiansToRotations(position_rad));
    }

    /**
     * Gets the current position of the roller mechanism in radians
     *
     * @return the position in radians
     */
    public double getCurrentPosition() {
        return inputs_.position;
    }

    /**
     * Gets the current velocity of the roller mechanism in radians per second
     *
     * @return the velocity in radians per second
     */
    public double getCurrentVelocity() {
        return inputs_.velocity;
    }

    /**
     * Gets the supply (battery-side) current draw of the leader motor in amps.
     *
     * @return the leader supply current in amps
     */
    public double getLeaderSupplyCurrent() {
        return inputs_.supplyCurrentDraw[0];
    }

    /**
     * Gets the stator (motor-winding) current draw of the leader motor in amps.
     *
     * @return the leader stator current in amps
     */
    public double getLeaderStatorCurrent() {
        return inputs_.statorCurrentDraw[0];
    }

    /**
     * Sets the target position of the roller mechanism in radians using standard position control
     *
     * @param position_rad the target position in radians
     */
    public void setTargetPosition(double position_rad) {
        position_target_ = position_rad;
        control_mode_ = ControlMode.POSITION;
        position_request_.Position = Units.radiansToRotations(position_rad);
        position_request_.FeedForward = 0.0; // Clear any feed forward
    }

    /**
     * Sets the target position of the roller mechanism with arbitrary feed forward.
     * This allows additional control output while holding a position (e.g., for a hood that adds backspin).
     *
     * @param position_rad the target position in radians
     * @param arbitrary_feedforward arbitrary feed forward value (units depend on slot gains configuration)
     */
    public void setTargetPositionWithFF(double position_rad, double arbitrary_feedforward) {
        position_target_ = position_rad;
        control_mode_ = ControlMode.POSITION;
        position_request_.Position = Units.radiansToRotations(position_rad);
        position_request_.FeedForward = arbitrary_feedforward;
    }

    /**
     * Sets the target position of the roller mechanism in radians using motion profile control
     *
     * @param position_rad the target position in radians
     */
    public void setTargetPositionMotionProfile(double position_rad) {
        position_target_ = position_rad;
        control_mode_ = ControlMode.MOTION_PROFILE_POSITION;
        motion_magic_position_request_.Position = Units.radiansToRotations(position_rad);
        motion_magic_position_request_.FeedForward = 0.0; // Clear any feed forward
    }

    /**
     * Sets the target position of the roller mechanism with arbitrary feed forward using motion profile control.
     * This allows additional control output while holding a position (e.g., for a hood that adds backspin).
     *
     * @param position_rad the target position in radians
     * @param arbitrary_feedforward arbitrary feed forward value (units depend on slot gains configuration)
     */
    public void setTargetPositionMotionProfileWithFF(double position_rad, double arbitrary_feedforward) {
        position_target_ = position_rad;
        control_mode_ = ControlMode.MOTION_PROFILE_POSITION;
        motion_magic_position_request_.Position = Units.radiansToRotations(position_rad);
        motion_magic_position_request_.FeedForward = arbitrary_feedforward;
    }

    /**
     * Sets the target velocity of the roller mechanism in radians per second using standard velocity control
     *
     * @param velocity_rad_per_sec the target velocity in radians per second
     */
    public void setTargetVelocity(double velocity_rad_per_sec) {
        control_mode_ = ControlMode.VELOCITY;
        velocity_target_ = velocity_rad_per_sec;
        velocity_request_.Velocity = Units.radiansToRotations(velocity_rad_per_sec);
    }

    /**
     * Sets the target velocity of the roller mechanism in radians per second using motion profile velocity control
     *
     * @param velocity_rad_per_sec the target velocity in radians per second
     */
    public void setTargetVelocityMotionProfile(double velocity_rad_per_sec) {
        control_mode_ = ControlMode.MOTION_PROFILE_VELOCITY;
        velocity_target_ = velocity_rad_per_sec;
        motion_magic_velocity_request_.Velocity = Units.radiansToRotations(velocity_rad_per_sec);
    }

    /**
     * Sets the target duty cycle of the roller mechanism
     *
     * @param duty_cycle the target duty cycle (-1.0 to 1.0)
     */
    public void setTargetDutyCycle(double duty_cycle) {
        control_mode_ = ControlMode.DUTY_CYCLE;
        duty_cycle_target_ = duty_cycle;
        duty_cycle_request_.Output = duty_cycle;
    }

    public void setTargetCurrent(double current_amps) {
        control_mode_ = ControlMode.CURRENT;
        current_target_ = current_amps; // For logging purposes, since current control doesn't use duty cycle
    }

    /**
     * Sets the target current with a feedforward component.
     *
     * @param current_amps the target current in amps
     * @param feedforward the feedforward component in amps to add to the target current
     */
    public void setTargetCurrentWithFF(double current_amps, double feedforward) {
        control_mode_ = ControlMode.CURRENT;
        current_target_ = current_amps;
        current_target_FF_ = feedforward;
    }

    /**
     * Applies a load torque to the roller mechanism for simulation purposes.
     * This method should be called during the simulation update cycle to apply
     * external loads (like friction, compression forces, etc.) to the mechanism.
     *
     * @param torque_nm The load torque in Newton-meters (Nm) at the roller output shaft.
     *                  Positive values oppose motion in the positive direction.
     */
    public void applyLoadTorque(double torque_nm) {
        sim_load_torque_nm_ = torque_nm;
    }
    /**
     * Sets the current limits for all motors in the mechanism.
     *
     * @param currentLimits the current limits configuration to apply
     */
    public void setCurrentLimits(CurrentLimitsConfigs currentLimits) {
        setMotorCurrentLimit(currentLimits, motors_);
    }
}
