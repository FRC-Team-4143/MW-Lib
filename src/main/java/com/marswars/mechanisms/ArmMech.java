package com.marswars.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;
import com.marswars.util.TunablePid;
import java.util.List;

/**
 * Mechanism implementation for a single-jointed arm with position, velocity, and duty control.
 */
public class ArmMech extends MechBase {

    /** Control modes for the arm mechanism */
    protected enum ControlMode {
        MOTION_MAGIC_POSITION,
        POSITION,
        VELOCITY,
        DUTY_CYCLE
    }

    private ControlMode control_mode_ = ControlMode.DUTY_CYCLE;

    // Always assume that we have the leader motor in index 0
    private final TalonFX motors_[];
    private final PositionVoltage position_request_;
    protected final MotionMagicVoltage motion_magic_position_request_;
    protected final boolean use_motion_magic_;
    private final VelocityVoltage velocity_request_;
    private final DutyCycleOut duty_cycle_request_;
    protected final BaseStatusSignal[] signals_;

    // Simulation
    private final SingleJointedArmSim arm_sim_;
    private final double gear_ratio_;
    private final DCMotor motor_type_;
    private final double moi_;
    private double sim_load_torque_nm_ = 0.0; // Load torque at arm shaft for simulation

    // sensor inputs
    protected double position_ = 0;
    protected double position_target_ = 0;
    protected double velocity_ = 0;
    protected double velocity_target_ = 0;
    protected double duty_cycle_target_ = 0;
    protected double[] applied_voltage_;
    protected double[] current_draw_;
    protected double[] motor_temp_c_;
    protected double[] bus_voltage_;

    /**
     * Constructs a new ArmMech
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs  List of motor configurations
     * @param gear_ratio     Gear ratio as motor rotations / mechanism rotations
     * @param length         Length of the arm in meters (Simulation only)
     * @param mass_kg        Mass of the arm in kg (Simulation only)
     * @param min_angle      Minimum angle of the arm in radians (Simulation only)
     * @param max_angle      Maximum angle of the arm in radians (Simulation only)
     *
     * @apiNote This constructor enables gravity compensation by default
     */
    public ArmMech(
            String logging_prefix,
            List<FxMotorConfig> motor_configs,
            double gear_ratio,
            double length,
            double mass_kg,
            double min_angle,
            double max_angle) {
        this(logging_prefix, null, motor_configs, gear_ratio, length, mass_kg, min_angle, max_angle, true);
    }

    /**
     * Constructs a new ArmMech
     *
     * @param logging_prefix String prefix for logging
     * @param mech_name      Name of the mechanism
     * @param motor_configs  List of motor configurations
     * @param gear_ratio     Gear ratio as motor rotations / mechanism rotations
     * @param length         Length of the arm in meters (Simulation only)
     * @param mass_kg        Mass of the arm in kg (Simulation only)
     * @param min_angle      Minimum angle of the arm in radians (Simulation only)
     * @param max_angle      Maximum angle of the arm in radians (Simulation only)
     *
     * @apiNote This constructor enables gravity compensation by default
     */
    public ArmMech(
            String logging_prefix,
            String mech_name,
            List<FxMotorConfig> motor_configs,
            double gear_ratio,
            double length,
            double mass_kg,
            double min_angle,
            double max_angle) {
        this(logging_prefix, mech_name, motor_configs, gear_ratio, length, mass_kg, min_angle, max_angle, true);
    }

    /**
     * Constructs a new ArmMech
     *
     * @param logging_prefix     String prefix for logging
     * @param motor_configs      List of motor configurations
     * @param gear_ratio         Gear ratio as motor rotations / mechanism rotations
     * @param length             Length of the arm in meters (Simulation only)
     * @param mass_kg            Mass of the arm in kg (Simulation only)
     * @param min_angle          Minimum angle of the arm in radians (Simulation
     *                           only)
     * @param max_angle          Maximum angle of the arm in radians (Simulation
     *                           only)
     * @param gravity_compensate true to enable gravity compensation, false
     *                           otherwise
     */
    public ArmMech(
            String logging_prefix,
            List<FxMotorConfig> motor_configs,
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
     * Constructs a new ArmMech
     *
     * @param logging_prefix     String prefix for logging
     * @param mech_name          Name of the mechanism
     * @param motor_configs      List of motor configurations
     * @param gear_ratio         Gear ratio as motor rotations / mechanism rotations
     * @param length             Length of the arm in meters (Simulation only)
     * @param mass_kg            Mass of the arm in kg (Simulation only)
     * @param min_angle          Minimum angle of the arm in radians (Simulation
     *                           only)
     * @param max_angle          Maximum angle of the arm in radians (Simulation
     *                           only)
     * @param gravity_compensate true to enable gravity compensation, false
     *                           otherwise
     */
    public ArmMech(
            String logging_prefix,
            String mech_name,
            List<FxMotorConfig> motor_configs,
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

        // MW-Lib convention: gear_ratio is motor/mechanism
        // Phoenix convention: SensorToMechanismRatio = sensor/mechanism = motor/mechanism
        // These are the same, so we can use gear_ratio directly
        double sensor_to_mech_ratio = gear_ratio;
        
        ConstructedMotors configured_motors = configMotors(
                motor_configs,
                sensor_to_mech_ratio,
                (cfg) -> {
                    // Configure the motor for position & velocity control with gravity
                    // compensation
                    if (gravity_compensate) {
                        cfg.config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
                        cfg.config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
                        cfg.config.Slot2.GravityType = GravityTypeValue.Arm_Cosine;
                    }
                    return cfg;
                });
        motors_ = configured_motors.motors;
        signals_ = configured_motors.signals;

        this.gear_ratio_ = gear_ratio;
        this.use_motion_magic_ = motor_configs.get(0).use_motion_magic;

        // default the inputs
        position_ = 0;
        velocity_ = 0;
        applied_voltage_ = new double[motors_.length];
        current_draw_ = new double[motors_.length];
        motor_temp_c_ = new double[motors_.length];
        bus_voltage_ = new double[motors_.length];

        ////////////////////////
        /// SIMULATION SETUP ///
        ////////////////////////

        if (motor_configs.get(0).motor_type == FxMotorType.X60) {
            motor_type_ = DCMotor.getKrakenX60(motor_configs.size());
        } else if (motor_configs.get(0).motor_type == FxMotorType.X44) {
            motor_type_ = DCMotor.getKrakenX44(motor_configs.size());
        } else if (motor_configs.get(0).motor_type == FxMotorType.FALCON500) {
            motor_type_ = DCMotor.getFalcon500(motor_configs.size());
        } else {
            throw new IllegalArgumentException("Unsupported motor type");
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

        // Setup tunable PIDs
        TunablePid.create(
                getLoggingKey() + "PositionGains",
                this::configPositionSlot,
                SlotConfigs.from(motor_configs.get(0).config.Slot0));
        DogLog.tunable(
                getLoggingKey() + "PositionGains/Setpoint", 0.0, (val) -> setTargetPosition(val));
        TunablePid.create(
                getLoggingKey() + "VelocityGains",
                this::configVelocitySlot,
                SlotConfigs.from(motor_configs.get(0).config.Slot1));
        DogLog.tunable(
                getLoggingKey() + "VelocityGains/Setpoint", 0.0, (val) -> setTargetVelocity(val));
        DogLog.tunable(
                getLoggingKey() + "DutyCycle/Setpoint", 0.0, (val) -> setTargetDutyCycle(val));
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
        }

        // run the simulation update step here if we are simulating
        if (IS_SIM) {
            // Provide a battery voltage to the TalonFX sim so controller output is
            // meaningful
            for (int i = 0; i < motors_.length; i++) {
                motors_[i].getSimState().setSupplyVoltage(12.0);
            }

            // Get the voltage the motor controller wants to apply
            double controller_voltage = motors_[0].getSimState().getMotorVoltage();
            
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
                motors_[i].getSimState().setRawRotorPosition(motorPosition);
                motors_[i].getSimState().setRotorVelocity(motorVelocity);
            }
        }
    }

    /** {@inheritDoc} */
    @Override
    public void writeOutputs(double timestamp) {
        switch (control_mode_) {
            case MOTION_MAGIC_POSITION:
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
            default:
                throw new IllegalStateException("Unexpected control mode: " + control_mode_);
        }
    }

    /** {@inheritDoc} */
    @Override
    public void logData() {
        // commands
        DogLog.log(getLoggingKey() + "control/mode", control_mode_.toString());
        DogLog.log(getLoggingKey() + "control/position/target", position_target_);
        DogLog.log(getLoggingKey() + "control/position/actual", position_);
        DogLog.log(getLoggingKey() + "control/velocity/target", velocity_target_);
        DogLog.log(getLoggingKey() + "control/velocity/actual", velocity_);
        DogLog.log(getLoggingKey() + "control/duty_cycle/target", duty_cycle_target_);
        DogLog.log(getLoggingKey() + "control/duty_cycle/actual", applied_voltage_[0] / 12.0);

        // per motor data
        for (int i = 0; i < motors_.length; i++) {
            DogLog.log(getLoggingKey() + "motor" + i + "/applied_voltage", applied_voltage_[i]);
            DogLog.log(getLoggingKey() + "motor" + i + "/current_draw", current_draw_[i]);
            DogLog.log(getLoggingKey() + "motor" + i + "/temp_c", motor_temp_c_[i]);
            DogLog.log(getLoggingKey() + "motor" + i + "/bus_voltage", bus_voltage_[i]);
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
            motors_[0].getConfigurator().apply(Slot0Configs.from(config));
        } else if (slot == 1) {
            motors_[0].getConfigurator().apply(Slot1Configs.from(config));
        } else {
            throw new IllegalArgumentException("Slot must be 0, 1, or 2");
        }
    }

    /**
     * Set the current position of the arm (for zeroing)
     *
     * @param position_rad the current position in radians
     */
    public void setCurrentPosition(double position_rad) {
        motors_[0].setPosition(Units.radiansToRotations(position_rad));
    }

    /**
     * Get the current position of the arm
     *
     * @return the current position in radians
     */
    public double getCurrentPosition() {
        return position_;
    }

    /**
     * Get the current velocity of the arm
     *
     * @return the current velocity in radians per second
     */
    public double getCurrentVelocity() {
        return velocity_;
    }

    /**
     * Get the current draw of the leader motor
     *
     * @return the current draw in amps
     */
    public double getLeaderCurrent() {
        return current_draw_[0];
    }

    /**
     * Set the target position of the arm
     *
     * @param position_rad the target position in radians
     */
    public void setTargetPosition(double position_rad) {
        position_target_ = position_rad;
        if (use_motion_magic_) {
            control_mode_ = ControlMode.MOTION_MAGIC_POSITION;
            motion_magic_position_request_.Position = Units.radiansToRotations(position_rad);
        } else {
            control_mode_ = ControlMode.POSITION;
            position_request_.Position = Units.radiansToRotations(position_rad);
        }
    }

    /**
     * Set the target velocity of the arm
     *
     * @param velocity_rad_per_sec the target velocity in radians per second
     */
    public void setTargetVelocity(double velocity_rad_per_sec) {
        control_mode_ = ControlMode.VELOCITY;
        velocity_target_ = velocity_rad_per_sec;
        velocity_request_.Velocity = Units.radiansToRotations(velocity_rad_per_sec);
    }

    /**
     * Set the target duty cycle of the arm
     *
     * @param duty_cycle the target duty cycle (-1.0 to 1.0)
     */
    public void setTargetDutyCycle(double duty_cycle) {
        control_mode_ = ControlMode.DUTY_CYCLE;
        duty_cycle_target_ = duty_cycle;
        duty_cycle_request_.Output = duty_cycle;
    }

    /**
     * Applies a load torque to the arm mechanism for simulation purposes.
     * This method should be called during the simulation update cycle to apply
     * external loads (like friction, compression forces, etc.) to the mechanism.
     *
     * @param torque_nm The load torque in Newton-meters (Nm) at the arm output shaft.
     *                  Positive values oppose motion in the positive direction.
     */
    public void applyLoadTorque(double torque_nm) {
        sim_load_torque_nm_ = torque_nm;
    }
}
