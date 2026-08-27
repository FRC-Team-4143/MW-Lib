package com.marswars.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.marswars.logging.MwLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.util.TunablePid;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Mechanism implementation for elevators with position, velocity, and duty cycle control.
 */
public class ElevatorMech extends MechBase {

    /** Control modes for the elevator mechanism */
    protected enum ControlMode {
        MOTION_PROFILE_POSITION,
        POSITION,
        VELOCITY,
        DUTY_CYCLE,
        CURRENT
    }

    private ControlMode control_mode_ = ControlMode.DUTY_CYCLE;

    // Temperature threshold for alerts (Celsius)
    private static final double MOTOR_TEMP_THRESHOLD_C = 65.0;

    // Always assume that we have the leader motor in index 0
    protected final CommonTalon motors_[];
    protected final PositionVoltage position_request_;
    protected final MotionMagicVoltage motion_magic_position_request_;
    protected final VelocityVoltage velocity_request_;
    protected final DutyCycleOut duty_cycle_request_;
    private final DutyCycleOut current_request_;
    protected final BaseStatusSignal[] signals_;
    private final PIDController current_pid_;
    private final LinearFilter current_filter_;

    // Alerts for motor monitoring
    protected final Alert[] motor_disconnected_alerts_;
    protected final Alert[] motor_temp_alerts_;
    protected final Debouncer[] motor_conn_debouncers_;

    // Simulation
    private final ElevatorSim elevator_sim_;
    private final double gear_ratio_;
    private final double drum_radius_;
    private final double position_to_rotations_;
    private final double rotations_to_position_;
    private final DCMotor motor_type_;
    private double sim_load_torque_nm_ = 0.0; // Load torque at drum shaft for simulation
    private final Mechanism2d mech2d_;
    private final MechanismLigament2d elevator_ligament_;

    // Command targets (outputs — stay as local fields)
    protected double position_target_ = 0;
    protected double velocity_target_ = 0;
    protected double duty_cycle_target_ = 0;
    protected double current_target_ = 0;
    protected double current_target_FF_ = 0;
    protected double filtered_torque_current_ = 0;

    // AdvantageKit inputs struct — sensor reads captured in the log for deterministic replay
    protected final MechInputsAutoLogged inputs_ = new MechInputsAutoLogged();

    /**
     * Constructs a new ElevatorMech (assumes vertical elevator)
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs List of motor configurations
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param drum_radius Radius of the drum in meters
     * @param carriage_mass_kg Mass of the elevator carriage in kg (Simulation only)
     * @param max_extension Maximum extension of the elevator in meters (Simulation only)
     */
    public ElevatorMech(
            String logging_prefix,
            List<MotorConfig> motor_configs,
            double gear_ratio,
            double drum_radius,
            double carriage_mass_kg,
            double max_extension) {
        this(
                logging_prefix,
                motor_configs,
                gear_ratio,
                drum_radius,
                carriage_mass_kg,
                max_extension,
                1.0);
    }

    /**
     * Constructs a new ElevatorMech
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs List of motor configurations
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param drum_radius Radius of the drum in meters
     * @param carriage_mass_kg Mass of the elevator carriage in kg (Simulation only)
     * @param max_extension Maximum extension of the elevator in meters (Simulation only)
     * @param rigging_ratio Rigging ratio of the elevator
     */
    public ElevatorMech(
            String logging_prefix,
            List<MotorConfig> motor_configs,
            double gear_ratio,
            double drum_radius,
            double carriage_mass_kg,
            double max_extension,
            double rigging_ratio) {
        this(
                logging_prefix,
                null,
                motor_configs,
                gear_ratio,
                drum_radius,
                carriage_mass_kg,
                max_extension,
                rigging_ratio,
                true);
    }

    /**
     * Constructs a new ElevatorMech
     *
     * @param logging_prefix String prefix for logging
     * @param mech_name Name of the mechanism
     * @param motor_configs List of motor configurations
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param drum_radius Radius of the drum in meters
     * @param carriage_mass_kg Mass of the elevator carriage in kg (Simulation only)
     * @param max_extension Maximum extension of the elevator in meters (Simulation only)
     * @param rigging_ratio Rigging ratio of the elevator
     */
    public ElevatorMech(
            String logging_prefix,
            String mech_name,
            List<MotorConfig> motor_configs,
            double gear_ratio,
            double drum_radius,
            double carriage_mass_kg,
            double max_extension,
            double rigging_ratio) {
        this(
                logging_prefix,
                mech_name,
                motor_configs,
                gear_ratio,
                drum_radius,
                carriage_mass_kg,
                max_extension,
                rigging_ratio,
                true);
    }

    /**
     * Constructs a new ElevatorMech
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs List of motor configurations
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param drum_radius Radius of the drum in meters
     * @param carriage_mass_kg Mass of the elevator carriage in kg (Simulation only)
     * @param max_extension Maximum extension of the elevator in meters (Simulation only)
     * @param rigging_ratio Rigging ratio of the elevator
     * @param is_vertical Whether the elevator is vertical (affects gravity compensation in
     *     simulation)
     */
    public ElevatorMech(
            String logging_prefix,
            List<MotorConfig> motor_configs,
            double gear_ratio,
            double drum_radius,
            double carriage_mass_kg,
            double max_extension,
            double rigging_ratio,
            boolean is_vertical) {
        this(
                logging_prefix,
                null,
                motor_configs,
                gear_ratio,
                drum_radius,
                carriage_mass_kg,
                max_extension,
                rigging_ratio,
                is_vertical);
    }

    /**
     * Constructs a new ElevatorMech
     *
     * @param logging_prefix String prefix for logging
     * @param mech_name Name of the mechanism
     * @param motor_configs List of motor configurations
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param drum_radius Radius of the drum in meters
     * @param carriage_mass_kg Mass of the elevator carriage in kg (Simulation only)
     * @param max_extension Maximum extension of the elevator in meters (Simulation only)
     * @param rigging_ratio Rigging ratio of the elevator
     * @param is_vertical Whether the elevator is vertical (affects gravity compensation in
     *     simulation)
     */
    public ElevatorMech(
            String logging_prefix,
            String mech_name,
            List<MotorConfig> motor_configs,
            double gear_ratio,
            double drum_radius,
            double carriage_mass_kg,
            double max_extension,
            double rigging_ratio,
            boolean is_vertical) {
        super(logging_prefix, mech_name);

        position_request_ = new PositionVoltage(0).withSlot(0);
        motion_magic_position_request_ = new MotionMagicVoltage(0).withSlot(0);
        velocity_request_ = new VelocityVoltage(0).withSlot(1);
        duty_cycle_request_ = new DutyCycleOut(0);
        current_request_ = new DutyCycleOut(0);

        // MW-Lib convention: gear_ratio is motor/mechanism
        // Phoenix convention: SensorToMechanismRatio = sensor/mechanism = motor/mechanism
        // These are the same, so we can use gear_ratio directly
        double sensor_to_mech_ratio = gear_ratio;
        
        // load the motors
        MechBase.ConstructedMotors configured_motors =
                configMotors(
                        motor_configs,
                        sensor_to_mech_ratio,
                        (cfg) -> {
                            // Configure the motor for position & velocity control with gravity
                            // compensation
                            if (cfg.isFXS()) {
                                TalonFXSConfiguration fxsConfig = cfg.getAsFXSConfig();
                                fxsConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
                                fxsConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
                                fxsConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;

                                // set the kG value if we are not vertical
                                if (!is_vertical) {
                                    fxsConfig.Slot0.kG = 0;
                                    fxsConfig.Slot1.kG = 0;
                                    fxsConfig.Slot2.kG = 0;
                                }
                            } else {
                                TalonFXConfiguration fxConfig = cfg.getAsFXConfig();
                                fxConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
                                fxConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
                                fxConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;

                                // set the kG value if we are not vertical
                                if (!is_vertical) {
                                    fxConfig.Slot0.kG = 0;
                                    fxConfig.Slot1.kG = 0;
                                    fxConfig.Slot2.kG = 0;
                                }
                            }

                            return cfg;
                        });
        motors_ = configured_motors.motors;
        signals_ = configured_motors.signals;

        this.gear_ratio_ = gear_ratio;
        this.drum_radius_ = drum_radius;
        this.position_to_rotations_ = 1 / (2.0 * Math.PI * drum_radius_);
        this.rotations_to_position_ = 2.0 * Math.PI * drum_radius_;

        // size the input arrays to motor count
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

        // construct the simulation object
        elevator_sim_ =
                new ElevatorSim(
                        motor_type_, // Motor type
                        gear_ratio,
                        carriage_mass_kg, // Carriage mass (kg)
                        drum_radius, // Drum radius (m)
                        0,
                        max_extension, // Max height (m)
                        is_vertical, // Simulate gravity
                        0 // Starting height (m)
                        );
        mech2d_ = new Mechanism2d(0.5 ,max_extension + 0.5);
        elevator_ligament_ = mech2d_.getRoot("Base", 0.25, 0).append(new MechanismLigament2d("Elevator", 0, 90));
        SmartDashboard.putData(getLoggingKey() + "mech2d", mech2d_);

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

            inputs_.position = rotations_to_position_ * motors_[0].getPosition().getValue().in(Rotations);
            inputs_.velocity =
                    rotations_to_position_ * motors_[0].getVelocity().getValue().in(RotationsPerSecond);
            for (int i = 0; i < motors_.length; i++) {
                inputs_.appliedVoltage[i] = motors_[i].getMotorVoltage().getValueAsDouble();
                inputs_.supplyCurrentDraw[i] = motors_[i].getSupplyCurrent().getValue().in(Amps);
                inputs_.statorCurrentDraw[i] = motors_[i].getStatorCurrent().getValue().in(Amps);
                inputs_.torqueCurrentDraw[i] = motors_[i].getTorqueCurrent().getValue().in(Amps);
                inputs_.motorTempC[i] = motors_[i].getDeviceTemp().getValue().in(Celsius);
                inputs_.busVoltage[i] = motors_[i].getSupplyVoltage().getValueAsDouble();

                // Update alerts for each motor
                motor_disconnected_alerts_[i].set(!motor_conn_debouncers_[i].calculate(motors_[i].isConnected()));
                motor_temp_alerts_[i].set(inputs_.motorTempC[i] > MOTOR_TEMP_THRESHOLD_C);
            }

            // run the simulation update step here if we are simulating
            if (IS_SIM) {
                // Get the voltage the motor controller wants to apply
                double controller_voltage = 0.0;
                if (motors_[0] instanceof TalonFX) {
                    controller_voltage = ((TalonFX) motors_[0]).getSimState().getMotorVoltage();
                } else if (motors_[0] instanceof TalonFXS) {
                    controller_voltage = ((TalonFXS) motors_[0]).getSimState().getMotorVoltage();
                }

                // Calculate the torque required to overcome the load at the motor shaft
                // (load torque at drum * gear ratio = load torque at motor)
                double motor_load_torque = sim_load_torque_nm_ * gear_ratio_;

                // Calculate the current needed to produce this load torque
                double load_current = motor_load_torque / motor_type_.KtNMPerAmp;

                // The voltage actually seen by the motor after the load consumes some current
                // is reduced by the voltage drop across the resistance due to load current
                double effective_voltage = controller_voltage - (load_current * motor_type_.rOhms);

                // Apply the effective voltage to the simulation
                elevator_sim_.setInput(effective_voltage);

                // Update simulation by 20ms
                elevator_sim_.update(0.020);

                // Reset the load torque after applying it (impulse load)
                // This must be called again each cycle for sustained load
                sim_load_torque_nm_ = 0.0;

                // Convert mechanism position to motor position
                // position_to_rotations_ converts meters to mechanism rotations
                // gear_ratio_ = motor/mechanism, so motor = mechanism * gear_ratio_
                double mechanismRotations = elevator_sim_.getPositionMeters() * position_to_rotations_;
                double mechanismRotationsPerSec =
                        elevator_sim_.getVelocityMetersPerSecond() * position_to_rotations_;

                double motorPosition = mechanismRotations * gear_ratio_;
                double motorVelocity = mechanismRotationsPerSec * gear_ratio_;

                for(int i = 0; i < motors_.length; i++) {
                    if (motors_[i] instanceof TalonFX) {
                        ((TalonFX) motors_[i]).getSimState().setRawRotorPosition(motorPosition);
                        ((TalonFX) motors_[i]).getSimState().setRotorVelocity(motorVelocity);
                    } else if (motors_[i] instanceof TalonFXS) {
                        ((TalonFXS) motors_[i]).getSimState().setRawRotorPosition(motorPosition);
                        ((TalonFXS) motors_[i]).getSimState().setRotorVelocity(motorVelocity);
                    }

                    // Simulation is always "connected" and at safe temperature
                    motor_disconnected_alerts_[i].set(false);
                    motor_temp_alerts_[i].set(false);
                }
            }
        }
        Logger.processInputs(getLoggingKey() + "Inputs", inputs_);
        elevator_ligament_.setLength(inputs_.position);
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
        MwLog.log(getLoggingKey() + "control/position/target", position_target_, Meters);
        MwLog.log(getLoggingKey() + "control/position/actual", inputs_.position, Meters);
        MwLog.log(getLoggingKey() + "control/velocity/target", velocity_target_, MetersPerSecond);
        MwLog.log(getLoggingKey() + "control/velocity/actual", inputs_.velocity, MetersPerSecond);
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
     * @param slot the slot to configure
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
     * Sets the current position of the elevator (for zeroing purposes)
     *
     * @param position the position to set in meters
     */
    public void setCurrentPosition(double position) {
        motors_[0].setPosition(position);
    }

    /**
     * Gets the current position of the elevator in meters
     *
     * @return the current position in meters
     */
    public double getCurrentPosition() {
        return inputs_.position;
    }

    /**
     * Gets the current velocity of the elevator in meters per second
     *
     * @return the current velocity in meters per second
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
     * Sets the target position of the elevator in meters using standard position control
     *
     * @param position_m the target position in meters
     */
    public void setTargetPosition(double position_m) {
        position_target_ = position_m;
        control_mode_ = ControlMode.POSITION;
        position_request_.Position = position_m * position_to_rotations_;
        position_request_.FeedForward = 0.0; // Clear any feed forward
    }

    /**
     * Sets the target position of the elevator with arbitrary feed forward.
     * This allows additional control output while holding a position.
     *
     * @param position_m the target position in meters
     * @param arbitrary_feedforward arbitrary feed forward value (units depend on slot gains configuration)
     */
    public void setTargetPositionWithFF(double position_m, double arbitrary_feedforward) {
        position_target_ = position_m;
        control_mode_ = ControlMode.POSITION;
        position_request_.Position = position_m * position_to_rotations_;
        position_request_.FeedForward = arbitrary_feedforward;
    }

    /**
     * Sets the target position of the elevator in meters using motion profile control
     *
     * @param position_m the target position in meters
     */
    public void setTargetPositionMotionProfile(double position_m) {
        position_target_ = position_m;
        control_mode_ = ControlMode.MOTION_PROFILE_POSITION;
        motion_magic_position_request_.Position = position_m * position_to_rotations_;
        motion_magic_position_request_.FeedForward = 0.0; // Clear any feed forward
    }

    /**
     * Sets the target position of the elevator with arbitrary feed forward using motion profile control.
     * This allows additional control output while holding a position.
     *
     * @param position_m the target position in meters
     * @param arbitrary_feedforward arbitrary feed forward value (units depend on slot gains configuration)
     */
    public void setTargetPositionMotionProfileWithFF(double position_m, double arbitrary_feedforward) {
        position_target_ = position_m;
        control_mode_ = ControlMode.MOTION_PROFILE_POSITION;
        motion_magic_position_request_.Position = position_m * position_to_rotations_;
        motion_magic_position_request_.FeedForward = arbitrary_feedforward;
    }

    /**
     * Sets the target velocity of the elevator in meters per second
     *
     * @param velocity_mps the target velocity in meters per second
     */
    public void setTargetVelocity(double velocity_mps) {
        control_mode_ = ControlMode.VELOCITY;
        velocity_target_ = velocity_mps;
        velocity_request_.Velocity = velocity_mps * position_to_rotations_;
    }

    /**
     * Sets the target duty cycle of the elevator
     *
     * @param duty_cycle the target duty cycle (-1.0 to 1.0)
     */
    public void setTargetDutyCycle(double duty_cycle) {
        control_mode_ = ControlMode.DUTY_CYCLE;
        duty_cycle_target_ = duty_cycle;
        duty_cycle_request_.Output = duty_cycle; 
    }

    /**
     * Sets the target current of the elevator in amps
     *
     * @param current_amps the target current in amps
     */
    public void setTargetCurrent(double current_amps) {
        control_mode_ = ControlMode.CURRENT;
        current_target_ = current_amps;
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
     * Applies a load torque to the elevator mechanism for simulation purposes.
     * This method should be called during the simulation update cycle to apply
     * external loads (like friction, compression forces, etc.) to the mechanism.
     *
     * @param torque_nm The load torque in Newton-meters (Nm) at the drum output shaft.
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
