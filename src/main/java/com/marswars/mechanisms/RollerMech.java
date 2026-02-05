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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;
import com.marswars.util.TunablePid;
import java.util.List;

public class RollerMech extends MechBase {

    /** Control modes for the roller mechanism */
    protected enum ControlMode {
        POSITION,
        VELOCITY,
        DUTY_CYCLE
    }

    private ControlMode control_mode_ = ControlMode.DUTY_CYCLE;

    // Always assume that we have the leader motor in index 0
    private final TalonFX motors_[];
    private final PositionVoltage position_request_;
    private final VelocityVoltage velocity_request_;
    private final DutyCycleOut duty_cycle_request_;
    protected final BaseStatusSignal[] signals_;

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


    // System parameters
    private final double gear_ratio_;
    private final double roller_inertia_;

    // Simulation
    private final DCMotor motor_type_;
    private final DCMotorSim roller_sim_;
    private double sim_load_torque_nm_ = 0.0; // Load torque at roller shaft for simulation

    public RollerMech(String logging_prefix, List<FxMotorConfig> motor_configs, double gear_ratio) {
        this(logging_prefix, null, motor_configs, gear_ratio, 0.00001);
    }

    /**
     * Constructs a new RollerMech
     *
     * @param logging_prefix String prefix for logging
     * @param mech_name Name of the mechanism
     * @param motor_configs Configuration for the roller motor
     * @param gear_ratio Gear ratio from motor TO roller
     */
    public RollerMech(String logging_prefix, String mech_name, List<FxMotorConfig> motor_configs, double gear_ratio) {
        this(logging_prefix, mech_name, motor_configs, gear_ratio, 0.00001);
    }

    /**
     * Constructs a new RollerMech
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs Configuration for the roller motor
     * @param gear_ratio Gear ratio from motor TO roller
     * @param roller_inertia Inertia of the roller in kg*m^2 (Simulation only)
     */
    public RollerMech(String logging_prefix, List<FxMotorConfig> motor_configs, double gear_ratio, double roller_inertia) {
        this(logging_prefix, null, motor_configs, gear_ratio, roller_inertia);
    }

    /**
     * Constructs a new RollerMech
     *
     * @param logging_prefix String prefix for logging
     * @param mech_name Name of the mechanism
     * @param motor_configs Configuration for the roller motor
     * @param gear_ratio Gear ratio from motor TO roller
     * @param roller_inertia Inertia of the roller in kg*m^2 (Simulation only)
     */
    public RollerMech(String logging_prefix, String mech_name, List<FxMotorConfig> motor_configs, double gear_ratio, double roller_inertia) {
        super(logging_prefix, mech_name);

        gear_ratio_ = gear_ratio;
        roller_inertia_ = roller_inertia;

        // MW-Lib convention: gear_ratio is "from motor TO mechanism" = mechanism/motor
        // Phoenix convention: SensorToMechanismRatio = sensor/mechanism = motor/mechanism
        // Therefore we need the inverse
        double sensor_to_mech_ratio = 1.0 / gear_ratio_;
        
        ConstructedMotors configured_motors = configMotors(motor_configs, sensor_to_mech_ratio);

        // Store system parameters
        position_request_ = new PositionVoltage(0).withSlot(0);
        velocity_request_ = new VelocityVoltage(0).withSlot(1);
        duty_cycle_request_ = new DutyCycleOut(0);

        // convert the list to an array for easy access
        motors_ = configured_motors.motors;
        signals_ = configured_motors.signals;

        // Setup Followers
        for (int i = 1; i < motors_.length; i++) {
            motors_[i].setControl(new StrictFollower(motors_[0].getDeviceID()));
        }

        // default the inputs
        position_ = 0;
        velocity_ = 0;
        applied_voltage_ = new double[motors_.length];
        current_draw_ = new double[motors_.length];
        motor_temp_c_ = new double[motors_.length];
        bus_voltage_ = new double[motors_.length];

        //////////////////////////
        /// SIMULATION SETUP ///
        //////////////////////////

        if (motor_configs.get(0).motor_type == FxMotorType.X60) {
            motor_type_ = DCMotor.getKrakenX60(motor_configs.size());
        } else if (motor_configs.get(0).motor_type == FxMotorType.X44) {
            motor_type_ = DCMotor.getKrakenX44(motor_configs.size());
        } else if (motor_configs.get(0).motor_type == FxMotorType.FALCON500) {
            motor_type_ = DCMotor.getFalcon500(motor_configs.size());
        } else {
            throw new IllegalArgumentException("Unsupported motor type");
        }

        roller_sim_ =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                motor_type_, roller_inertia_, 1.0 / gear_ratio_),
                        motor_type_);

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
            // (load torque at roller / gear ratio = load torque at motor)
            double motor_load_torque = sim_load_torque_nm_ / gear_ratio_;
            
            // Calculate the current needed to produce this load torque
            double load_current = motor_load_torque / motor_type_.KtNMPerAmp;
            
            // The voltage actually seen by the motor after the load consumes some current
            // is reduced by the voltage drop across the resistance due to load current
            double effective_voltage = controller_voltage - (load_current * motor_type_.rOhms);
            
            // Apply the effective voltage to the simulation
            roller_sim_.setInput(effective_voltage);

            // Update simulation by 20ms
            roller_sim_.update(0.020);

            // Reset the load torque after applying it (impulse load)
            // This must be called again each cycle for sustained load
            sim_load_torque_nm_ = 0.0;

            // The simulation gives mechanism (output) position/velocity in radians
            // setRawRotorPosition expects raw rotor (motor) position in rotations
            // Since gear_ratio_ = mechanism/motor, we need motor = mechanism / gear_ratio_
            double mechanismPositionRad = roller_sim_.getAngularPositionRad();
            double mechanismVelocityRadPerSec = roller_sim_.getAngularVelocityRadPerSec();
            
            double motorPositionRad = mechanismPositionRad / gear_ratio_;
            double motorVelocityRadPerSec = mechanismVelocityRadPerSec / gear_ratio_;
            
            double motorPosition = Radians.of(motorPositionRad).in(Rotations);
            double motorVelocity = RadiansPerSecond.of(motorVelocityRadPerSec).in(RotationsPerSecond);

            for(int i = 0; i < motors_.length; i++) {
                motors_[i].getSimState().setRawRotorPosition(motorPosition);
                motors_[i].getSimState().setRotorVelocity(motorVelocity);
            }
        }
    }

    @Override
    public void writeOutputs(double timestamp) {
        switch (control_mode_) {
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

        DogLog.log(getLoggingKey() + "motor/applied_voltage", applied_voltage_);
        DogLog.log(getLoggingKey() + "motor/current_draw", current_draw_);
        DogLog.log(getLoggingKey() + "motor/temp_c", motor_temp_c_);
        DogLog.log(getLoggingKey() + "motor/bus_voltage", bus_voltage_);
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
            motors_[0].getConfigurator().apply(Slot0Configs.from(config));
        } else if (slot == 1) {
            motors_[0].getConfigurator().apply(Slot1Configs.from(config));
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
        return position_;
    }

    /**
     * Gets the current velocity of the roller mechanism in radians per second
     *
     * @return the velocity in radians per second
     */
    public double getCurrentVelocity() {
        return velocity_;
    }

    /**
     * Gets the current draw of the leader motor in amps
     *
     * @return the current draw in amps
     */
    public double getLeaderCurrent() {
        return current_draw_[0];
    }

    /**
     * Sets the target position of the roller mechanism in radians
     *
     * @param position_rad the target position in radians
     */
    public void setTargetPosition(double position_rad) {
        position_target_ = position_rad;
        control_mode_ = ControlMode.POSITION;
        position_request_.Position = Units.radiansToRotations(position_rad);
    }

    /**
     * Sets the target velocity of the roller mechanism in radians per second
     *
     * @param velocity_rad_per_sec the target velocity in radians per second
     */
    public void setTargetVelocity(double velocity_rad_per_sec) {
        control_mode_ = ControlMode.VELOCITY;
        velocity_target_ = velocity_rad_per_sec;
        velocity_request_.Velocity = Units.radiansToRotations(velocity_rad_per_sec);
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
}
