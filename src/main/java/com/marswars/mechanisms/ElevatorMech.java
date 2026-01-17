package com.marswars.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;
import com.marswars.util.TunablePid;
import java.util.List;

public class ElevatorMech extends MechBase {

    /** Control modes for the elevator mechanism */
    protected enum ControlMode {
        MOTION_MAGIC_POSITION,
        POSITION,
        VELOCITY,
        DUTY_CYCLE
    }

    protected ControlMode control_mode_ = ControlMode.DUTY_CYCLE;

    // Always assume that we have the leader motor in index 0
    protected final TalonFX motors_[];
    protected final PositionVoltage position_request_;
    protected final MotionMagicVoltage motion_magic_position_request_;
    protected final boolean use_motion_magic_;
    protected final VelocityVoltage velocity_request_;
    protected final DutyCycleOut duty_cycle_request_;
    protected final BaseStatusSignal[] signals_;

    // Simulation
    private final ElevatorSim elevator_sim_;
    private final double gear_ratio_;
    private final double drum_radius_;
    private final double position_to_rotations_;
    private final DCMotor motor_type_;

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
     * Constructs a new ElevatorMech (assumes vertical elevator)
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs List of motor configurations
     * @param gear_ratio Gear ratio from motor TO drum
     * @param drum_radius Radius of the drum in meters
     * @param carriage_mass_kg Mass of the elevator carriage in kg (Simulation only)
     * @param max_extension Maximum extension of the elevator in meters (Simulation only)
     * @param rigging_ratio Rigging ratio of the elevator
     */
    public ElevatorMech(
            String logging_prefix,
            List<FxMotorConfig> motor_configs,
            double gear_ratio,
            double drum_radius,
            double carriage_mass_kg,
            double max_extension,
            double rigging_ratio) {
        this(
                logging_prefix,
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
     * @param gear_ratio Gear ratio from motor TO drum
     * @param drum_radius Radius of the drum in meters
     * @param carriage_mass_kg Mass of the elevator carriage in kg (Simulation only)
     * @param max_extension Maximum extension of the elevator in meters (Simulation only)
     * @param rigging_ratio Rigging ratio of the elevator
     * @param is_vertical Whether the elevator is vertical (affects gravity compensation in
     *     simulation)
     */
    public ElevatorMech(
            String logging_prefix,
            List<FxMotorConfig> motor_configs,
            double gear_ratio,
            double drum_radius,
            double carriage_mass_kg,
            double max_extension,
            double rigging_ratio,
            boolean is_vertical) {
        super(logging_prefix);

        position_request_ = new PositionVoltage(0).withSlot(0);
        motion_magic_position_request_ = new MotionMagicVoltage(0).withSlot(0);
        velocity_request_ = new VelocityVoltage(0).withSlot(1);
        duty_cycle_request_ = new DutyCycleOut(0);

        // load the motors
        ConstructedMotors configured_motors =
                configMotors(
                        motor_configs,
                        gear_ratio,
                        (cfg) -> {
                            // Configure the motor for position & velocity control with gravity
                            // compensation
                            cfg.config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
                            cfg.config.Slot1.GravityType = GravityTypeValue.Elevator_Static;
                            cfg.config.Slot2.GravityType = GravityTypeValue.Elevator_Static;

                            // set the kG value if we are not vertical
                            if (!is_vertical) {
                                cfg.config.Slot0.kG = 0;
                                cfg.config.Slot1.kG = 0;
                                cfg.config.Slot2.kG = 0;
                            }

                            return cfg;
                        });
        motors_ = configured_motors.motors;
        signals_ = configured_motors.signals;

        this.gear_ratio_ = gear_ratio;
        this.drum_radius_ = drum_radius;
        this.position_to_rotations_ = 1 / (2.0 * Math.PI * drum_radius_);
        this.use_motion_magic_ = motor_configs.get(0).use_motion_magic;

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
        } else {
            throw new IllegalArgumentException("Unsupported motor type");
            // motor_type = DCMotor.getKr(motor_configs.size());
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
    }

    @Override
    public void readInputs(double timestamp) {
        BaseStatusSignal.refreshAll(signals_);

        // always read the sensor data
        position_ = position_to_rotations_ * motors_[0].getPosition().getValue().in(Rotations);
        velocity_ =
                position_to_rotations_ * motors_[0].getVelocity().getValue().in(RotationsPerSecond);
        for (int i = 0; i < motors_.length; i++) {
            applied_voltage_[i] = motors_[i].getMotorVoltage().getValueAsDouble();
            current_draw_[i] = motors_[i].getSupplyCurrent().getValue().in(Amps);
            motor_temp_c_[i] = motors_[i].getDeviceTemp().getValue().in(Celsius);
            bus_voltage_[i] = motors_[i].getSupplyVoltage().getValueAsDouble();
        }

        // run the simulation update step here if we are simulating
        if (IS_SIM) {
            // Set input voltage from motor controller to simulation
            elevator_sim_.setInput(motors_[0].getSimState().getMotorVoltage());

            // Update simulation by 20ms
            elevator_sim_.update(0.020);

            // Convert meters to motor rotations
            double motorPosition =
                    elevator_sim_.getPositionMeters() * position_to_rotations_ * gear_ratio_;
            double motorVelocity =
                    elevator_sim_.getVelocityMetersPerSecond()
                            * position_to_rotations_
                            * gear_ratio_;

            motors_[0].getSimState().setRawRotorPosition(motorPosition);
            motors_[0].getSimState().setRotorVelocity(motorVelocity);
        }
    }

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

    @Override
    public void logData() {
        // commands
        DogLog.log(getLoggingKey() + "control/mode", control_mode_.toString());
        DogLog.log(getLoggingKey() + "control/position/target", position_target_);
        DogLog.log(getLoggingKey() + "control/position/actual", position_);
        DogLog.log(getLoggingKey() + "control/velocity/target", velocity_target_);
        DogLog.log(getLoggingKey() + "control/velocity/actual", velocity_);
        DogLog.log(getLoggingKey() + "control/duty_cycle/target", duty_cycle_target_);

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
     * @param slot the slot to configure
     * @param config the slot config to apply
     */
    private void configSlot(int slot, SlotConfigs config) {
        if (slot == 0) {
            motors_[0].getConfigurator().apply(Slot0Configs.from(config));
        } else if (slot == 1) {
            motors_[0].getConfigurator().apply(Slot1Configs.from(config));
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
        return position_;
    }

    /**
     * Gets the current velocity of the elevator in meters per second
     *
     * @return the current velocity in meters per second
     */
    public double getCurrentVelocity() {
        return velocity_;
    }

    /**
     * Gets the current draw of the leader motor in amps
     *
     * @return the current draw of the leader motor in amps
     */
    public double getLeaderCurrent() {
        return current_draw_[0];
    }

    /**
     * Sets the target position of the elevator in meters
     *
     * @param position_m the target position in meters
     */
    public void setTargetPosition(double position_m) {
        position_target_ = position_m;
        if (use_motion_magic_) {
            control_mode_ = ControlMode.MOTION_MAGIC_POSITION;
            motion_magic_position_request_.Position = position_m / position_to_rotations_;
        } else {
            control_mode_ = ControlMode.POSITION;
            position_request_.Position = position_m / position_to_rotations_;
        }
    }

    /**
     * Sets the target velocity of the elevator in meters per second
     *
     * @param velocity_mps the target velocity in meters per second
     */
    public void setTargetVelocity(double velocity_mps) {
        control_mode_ = ControlMode.VELOCITY;
        velocity_target_ = velocity_mps;
        velocity_request_.Velocity = velocity_mps / position_to_rotations_;
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
}
