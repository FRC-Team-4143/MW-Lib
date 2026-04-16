package com.marswars.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.util.TunablePid;
import java.util.List;

/**
 * Mechanism implementation for a single-jointed arm with position, velocity, and duty control.
 */
public class ArmMech extends MechBase {

    /** Control modes for the arm mechanism */
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
    private final CommonTalon motors_[];
    private final PositionVoltage position_request_;
    protected final MotionMagicVoltage motion_magic_position_request_;
    private final VelocityVoltage velocity_request_;
    private final DutyCycleOut duty_cycle_request_;
    private final DutyCycleOut current_request_;
    protected final BaseStatusSignal[] signals_;
    private final PIDController current_pid_;

    // Alerts for motor monitoring
    protected final Alert[] motor_disconnected_alerts_;
    protected final Alert[] motor_temp_alerts_;
    protected final Debouncer[] motor_conn_debouncers_;

    // Simulation
    private final SingleJointedArmSim arm_sim_;
    private final double gear_ratio_;
    private final DCMotor motor_type_;
    private final double moi_;
    private double sim_load_torque_nm_ = 0.0; // Load torque at arm shaft for simulation
    private final Mechanism2d mech2d_;
    private final MechanismLigament2d arm_ligament_;

    // sensor inputs
    protected double position_ = 0;
    protected double position_target_ = 0;
    protected double velocity_ = 0;
    protected double velocity_target_ = 0;
    protected double duty_cycle_target_ = 0;
    protected double current_target_ = 0;
    protected double[] applied_voltage_;
    protected double[] current_draw_;
    protected double[] motor_temp_c_;
    protected double[] bus_voltage_;

    /**
     * Constructs a new FxArmMech with gravity compensation enabled by default
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs  List of motor configurations
     * @param gear_ratio     Gear ratio as motor rotations / mechanism rotations
     * @param length         Length of the arm in meters (Simulation only)
     * @param mass_kg        Mass of the arm in kg (Simulation only)
     * @param min_angle      Minimum angle of the arm in radians (Simulation only)
     * @param max_angle      Maximum angle of the arm in radians (Simulation only)
     */
    public ArmMech(
            String logging_prefix,
            List<MotorConfig> motor_configs,
            double gear_ratio,
            double length,
            double mass_kg,
            double min_angle,
            double max_angle) {
        this(logging_prefix, null, motor_configs, gear_ratio, length, mass_kg, min_angle, max_angle, true);
    }

    /**
     * Constructs a new FxArmMech with a mechanism name and gravity compensation enabled by default
     *
     * @param logging_prefix String prefix for logging
     * @param mech_name      Name of the mechanism
     * @param motor_configs  List of motor configurations
     * @param gear_ratio     Gear ratio as motor rotations / mechanism rotations
     * @param length         Length of the arm in meters (Simulation only)
     * @param mass_kg        Mass of the arm in kg (Simulation only)
     * @param min_angle      Minimum angle of the arm in radians (Simulation only)
     * @param max_angle      Maximum angle of the arm in radians (Simulation only)
     */
    public ArmMech(
            String logging_prefix,
            String mech_name,
            List<MotorConfig> motor_configs,
            double gear_ratio,
            double length,
            double mass_kg,
            double min_angle,
            double max_angle) {
        this(logging_prefix, mech_name, motor_configs, gear_ratio, length, mass_kg, min_angle, max_angle, true);
    }

    /**
     * Constructs a new FxArmMech with configurable gravity compensation
     *
     * @param logging_prefix     String prefix for logging
     * @param motor_configs      List of motor configurations
     * @param gear_ratio         Gear ratio as motor rotations / mechanism rotations
     * @param length             Length of the arm in meters (Simulation only)
     * @param mass_kg            Mass of the arm in kg (Simulation only)
     * @param min_angle          Minimum angle of the arm in radians (Simulation only)
     * @param max_angle          Maximum angle of the arm in radians (Simulation only)
     * @param gravity_compensate true to enable gravity compensation, false otherwise
     */
    public ArmMech(
            String logging_prefix,
            List<MotorConfig> motor_configs,
            double gear_ratio,
            double length,
            double mass_kg,
            double min_angle,
            double max_angle,
            boolean gravity_compensate) {
        this(logging_prefix, null, motor_configs, gear_ratio, length, mass_kg, min_angle, max_angle,
                gravity_compensate);

    }

    /**
     * Constructs a new FxArmMech with a mechanism name and configurable gravity compensation
     *
     * @param logging_prefix     String prefix for logging
     * @param mech_name          Name of the mechanism (used for multiple instances of the same mech)
     * @param motor_configs      List of motor configurations
     * @param gear_ratio         Gear ratio as motor rotations / mechanism rotations
     * @param length             Length of the arm in meters (Simulation only)
     * @param mass_kg            Mass of the arm in kg (Simulation only)
     * @param min_angle          Minimum angle of the arm in radians (Simulation only)
     * @param max_angle          Maximum angle of the arm in radians (Simulation only)
     * @param gravity_compensate true to enable gravity compensation, false otherwise
     */
    public ArmMech(
            String logging_prefix,
            String mech_name,
            List<MotorConfig> motor_configs,
            double gear_ratio,
            double length,
            double mass_kg,
            double min_angle,
            double max_angle,
            boolean gravity_compensate) {
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
        
        MechBase.ConstructedMotors configured_motors = 
                configMotors(
                motor_configs,
                sensor_to_mech_ratio,
                (cfg) -> {
                    // Configure the motor for position & velocity control with gravity
                    // compensation
                    if (gravity_compensate) {
                        if (cfg.isFXS()) {
                            TalonFXSConfiguration fxsConfig = cfg.getAsFXSConfig();
                            fxsConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
                            fxsConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
                            fxsConfig.Slot2.GravityType = GravityTypeValue.Arm_Cosine;
                        } else {
                            TalonFXConfiguration fxConfig = cfg.getAsFXConfig();
                            fxConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
                            fxConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
                            fxConfig.Slot2.GravityType = GravityTypeValue.Arm_Cosine;
                        }
                    }
                    return cfg;
                });
        motors_ = configured_motors.motors;
        signals_ = configured_motors.signals;

        this.gear_ratio_ = gear_ratio;

        // default the inputs
        position_ = 0;
        velocity_ = 0;
        applied_voltage_ = new double[motors_.length];
        current_draw_ = new double[motors_.length];
        motor_temp_c_ = new double[motors_.length];
        bus_voltage_ = new double[motors_.length];

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

        ////////////////////////
        /// SIMULATION SETUP ///
        ////////////////////////

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

        moi_ = SingleJointedArmSim.estimateMOI(length, mass_kg);
        arm_sim_ = new SingleJointedArmSim(
                motor_type_, // Motor type
                gear_ratio,
                moi_,
                length, // Length of the arm (meters)
                min_angle, // Minimum angle (radians)
                max_angle, // Maximum angle (radians)
                gravity_compensate, // Simulate gravity
                0 // Starting angle (radians)
        );
        mech2d_ = new Mechanism2d((length * 2) + 0.5 ,(length * 2) + 0.);
        arm_ligament_ = mech2d_.getRoot("Base", length, length).append(new MechanismLigament2d("Arm", length, min_angle));
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
        
        TunablePid.create(
                getLoggingKey() + "PositionGains",
                this::configPositionSlot,
                slot0Config);
        DogLog.tunable(
                getLoggingKey() + "PositionGains/Setpoint", 0.0, (val) -> setTargetPosition(val));
        TunablePid.create(
                getLoggingKey() + "VelocityGains",
                this::configVelocitySlot,
                slot1Config);
        DogLog.tunable(
                getLoggingKey() + "VelocityGains/Setpoint", 0.0, (val) -> setTargetVelocity(val));
        DogLog.tunable(
                getLoggingKey() + "DutyCycle/Setpoint", 0.0, (val) -> setTargetDutyCycle(val));
        TunablePid.create("CurrentGains", current_pid_);
        DogLog.tunable(
                getLoggingKey() + "Current/Setpoint", 0.0, (val) -> setTargetCurrent(val));
    }

    /** {@inheritDoc} */
    @Override
    public void readInputs(double timestamp) {
        BaseStatusSignal.refreshAll(signals_);

        // always read the sensor data
        position_ = motors_[0].getPosition().getValue().in(Radians);
        velocity_ = motors_[0].getVelocity().getValue().in(RadiansPerSecond);
        for (int i = 0; i < motors_.length; i++) {
            applied_voltage_[i] = motors_[i].getMotorVoltage().getValueAsDouble();
            current_draw_[i] = motors_[i].getSupplyCurrent().getValue().in(Amps);
            motor_temp_c_[i] = motors_[i].getDeviceTemp().getValue().in(Celsius);
            bus_voltage_[i] = motors_[i].getSupplyVoltage().getValueAsDouble();
            
            // Update alerts for each motor
            motor_disconnected_alerts_[i].set(!motor_conn_debouncers_[i].calculate(motors_[i].isConnected()));
            motor_temp_alerts_[i].set(motor_temp_c_[i] > MOTOR_TEMP_THRESHOLD_C);
        }
        arm_ligament_.setAngle(Units.radiansToDegrees(position_));

        // run the simulation update step here if we are simulating
        if (IS_SIM) {
            // Provide a battery voltage to the motor sim so controller output is meaningful
            for (int i = 0; i < motors_.length; i++) {
                if (motors_[i] instanceof TalonFX) {
                    ((TalonFX) motors_[i]).getSimState().setSupplyVoltage(12.0);
                } else if (motors_[i] instanceof TalonFXS) {
                    ((TalonFXS) motors_[i]).getSimState().setSupplyVoltage(12.0);
                }
            }

            // Get the voltage the motor controller wants to apply
            double controller_voltage = 0.0;
            if (motors_[0] instanceof TalonFX) {
                controller_voltage = ((TalonFX) motors_[0]).getSimState().getMotorVoltage();
            } else if (motors_[0] instanceof TalonFXS) {
                controller_voltage = ((TalonFXS) motors_[0]).getSimState().getMotorVoltage();
            }
            
            // Calculate the torque required to overcome the load at the motor shaft
            // (load torque at arm * gear ratio = load torque at motor)
            double motor_load_torque = sim_load_torque_nm_ * gear_ratio_;
            
            // Calculate the current needed to produce this load torque
            double load_current = motor_load_torque / motor_type_.KtNMPerAmp;
            
            // The voltage actually seen by the motor after the load consumes some current
            // is reduced by the voltage drop across the resistance due to load current
            double effective_voltage = controller_voltage - (load_current * motor_type_.rOhms);
            
            // Apply the effective voltage to the simulation
            arm_sim_.setInput(effective_voltage);

            // Update simulation by 20ms
            arm_sim_.update(0.020);

            // Reset the load torque after applying it (impulse load)
            // This must be called again each cycle for sustained load
            sim_load_torque_nm_ = 0.0;

            // Convert mechanism position to motor position
            // gear_ratio_ = motor/mechanism, so motor = mechanism * gear_ratio_
            double mechanismPositionRad = arm_sim_.getAngleRads();
            double mechanismVelocityRadPerSec = arm_sim_.getVelocityRadPerSec();
            
            double motorPositionRad = mechanismPositionRad * gear_ratio_;
            double motorVelocityRadPerSec = mechanismVelocityRadPerSec * gear_ratio_;
            
            double motorPosition = Radians.of(motorPositionRad).in(Rotations);
            double motorVelocity = RadiansPerSecond.of(motorVelocityRadPerSec).in(RotationsPerSecond);

            for (int i = 0; i < motors_.length; i++) {
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
                // For current control, we will use the PID controller to calculate the required voltage
                double current_error = current_target_ - current_draw_[0];
                double voltage_output = current_pid_.calculate(current_error);
                // Clamp the voltage output to the max voltage of the system (e.g., 12V)
                voltage_output = Math.max(-12.0, Math.min(12.0, voltage_output));
                current_request_.Output = voltage_output / 12.0; // Convert to duty cycle
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
        DogLog.log(getLoggingKey() + "control/mode", control_mode_.toString());
        DogLog.log(getLoggingKey() + "control/position/target", position_target_, Radians);
        DogLog.log(getLoggingKey() + "control/position/actual", position_, Radians);
        DogLog.log(getLoggingKey() + "control/velocity/target", velocity_target_, RadiansPerSecond);
        DogLog.log(getLoggingKey() + "control/velocity/actual", velocity_, RadiansPerSecond);
        DogLog.log(getLoggingKey() + "control/duty_cycle/target", duty_cycle_target_, Percent);
        DogLog.log(getLoggingKey() + "control/duty_cycle/actual", applied_voltage_[0] / 12.0, Percent);
        DogLog.log(getLoggingKey() + "control/current/target", current_target_, Amps);
        DogLog.log(getLoggingKey() + "control/current/actual", current_draw_[0], Amps);

        // per motor data
        for (int i = 0; i < motors_.length; i++) {
            DogLog.log(getLoggingKey() + "motor" + i + "/applied_voltage", applied_voltage_[i], Volts);
            DogLog.log(getLoggingKey() + "motor" + i + "/current_draw", current_draw_[i], Amps);
            DogLog.log(getLoggingKey() + "motor" + i + "/temp", motor_temp_c_[i], Celsius);
            DogLog.log(getLoggingKey() + "motor" + i + "/bus_voltage", bus_voltage_[i], Volts);
        }
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
     * @param slot   the slot to configure
     * @param config the config to apply
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
        } else {
            throw new IllegalArgumentException("Slot must be 0, 1, or 2");
        }
    }

    /** {@inheritDoc} */
    public void setCurrentPosition(double position_rad) {
        motors_[0].setPosition(Units.radiansToRotations(position_rad));
    }

    /** {@inheritDoc} */
    public double getCurrentPosition() {
        return position_;
    }

    /** {@inheritDoc} */
    public double getCurrentVelocity() {
        return velocity_;
    }

    /** {@inheritDoc} */
    public double getLeaderCurrent() {
        return current_draw_[0];
    }

    /** {@inheritDoc} */
    public void setTargetPosition(double position_rad) {
        position_target_ = position_rad;
        control_mode_ = ControlMode.POSITION;
        position_request_.Position = Units.radiansToRotations(position_rad);
        position_request_.FeedForward = 0.0; // Clear any feed forward
    }

    /** {@inheritDoc} */
    public void setTargetPositionWithFF(double position_rad, double arbitrary_feedforward) {
        position_target_ = position_rad;
        control_mode_ = ControlMode.POSITION;
        position_request_.Position = Units.radiansToRotations(position_rad);
        position_request_.FeedForward = arbitrary_feedforward;
    }

    /** {@inheritDoc} */
    public void setTargetPositionMotionProfile(double position_rad) {
        position_target_ = position_rad;
        control_mode_ = ControlMode.MOTION_PROFILE_POSITION;
        motion_magic_position_request_.Position = Units.radiansToRotations(position_rad);
        motion_magic_position_request_.FeedForward = 0.0; // Clear any feed forward
    }

    /** {@inheritDoc} */
    public void setTargetPositionMotionProfileWithFF(double position_rad, double arbitrary_feedforward) {
        position_target_ = position_rad;
        control_mode_ = ControlMode.MOTION_PROFILE_POSITION;
        motion_magic_position_request_.Position = Units.radiansToRotations(position_rad);
        motion_magic_position_request_.FeedForward = arbitrary_feedforward;
    }

    /** {@inheritDoc} */
    public void setTargetVelocity(double velocity_rad_per_sec) {
        control_mode_ = ControlMode.VELOCITY;
        velocity_target_ = velocity_rad_per_sec;
        velocity_request_.Velocity = Units.radiansToRotations(velocity_rad_per_sec);
    }

    /** {@inheritDoc} */
    public void setTargetDutyCycle(double duty_cycle) {
        control_mode_ = ControlMode.DUTY_CYCLE;
        duty_cycle_target_ = duty_cycle;
        duty_cycle_request_.Output = duty_cycle;
    }

    /**
     * Sets the target current of the arm in amps
     *
     * @param current_amps the target current in amps
     */
    public void setTargetCurrent(double current_amps) {
        control_mode_ = ControlMode.CURRENT;
        current_target_ = current_amps;
    }

    /** {@inheritDoc} */
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
