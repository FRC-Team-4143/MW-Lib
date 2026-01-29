package com.marswars.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;
import com.marswars.util.TunablePid;
import java.util.List;

public class FlywheelMech extends MechBase {

    /** Control modes for the flywheel mechanism */
    protected enum ControlMode {
        VELOCITY,
        DUTY_CYCLE
    }

    private ControlMode control_mode_ = ControlMode.DUTY_CYCLE;

    // Always assume that we have the leader motor in index 0
    private final TalonFX motors_[];

    // control and status
    private final VelocityVoltage velocity_request_;
    private final DutyCycleOut duty_cycle_request_;
    protected final BaseStatusSignal[] signals_;

    // Simulation info
    protected final FlywheelSim flywheel_sim_;
    protected final double gear_ratio_;
    protected final double wheel_inertia_;
    protected final double wheel_radius_;
    private final DCMotor motor_type_;

    // Current state info
    protected double position_ = 0; // only used in sim
    protected double velocity_ = 0;
    protected double velocity_target_ = 0;
    protected double duty_cycle_target_ = 0;
    protected double[] applied_voltage_;
    protected double[] current_draw_;
    protected double[] motor_temp_c_;
    protected double[] bus_voltage_;

    /**
     * Constructs a new FlywheelMech
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs List of motor configurations
     * @param gear_ratio Gear ratio from motor TO wheel
     * @param wheel_inertia Inertia of the flywheel in kg*m^2 (Simulation only)
     * @param wheel_radius Radius of the flywheel in meters (Simulation only)
     */
    public FlywheelMech(
            String logging_prefix,
            List<FxMotorConfig> motor_configs,
            double gear_ratio,
            double wheel_inertia,
            double wheel_radius) {
        this(logging_prefix, null, motor_configs, gear_ratio, wheel_inertia, wheel_radius);
    }

    /**
     * Constructs a new FlywheelMech
     *
     * @param logging_prefix String prefix for logging
     * @param mech_name Name of the mechanism
     * @param motor_configs List of motor configurations
     * @param gear_ratio Gear ratio from motor TO wheel
     * @param wheel_inertia Inertia of the flywheel in kg*m^2 (Simulation only)
     * @param wheel_radius Radius of the flywheel in meters (Simulation only)
     */
    public FlywheelMech(
            String logging_prefix,
            String mech_name,
            List<FxMotorConfig> motor_configs,
            double gear_ratio,
            double wheel_inertia,
            double wheel_radius) {
        super(logging_prefix, mech_name);

        // Create control requests
        this.velocity_request_ = new VelocityVoltage(0).withSlot(1);
        this.duty_cycle_request_ = new DutyCycleOut(0);

        ConstructedMotors configured_motors = configMotors(motor_configs, gear_ratio);
        motors_ = configured_motors.motors;
        signals_ = configured_motors.signals;

        // set the system constants
        this.gear_ratio_ = gear_ratio;
        this.wheel_inertia_ = wheel_inertia;
        this.wheel_radius_ = wheel_radius;

        // Setup Followers
        for (int i = 1; i < motors_.length; i++) {
            motors_[i].setControl(new StrictFollower(motors_[0].getDeviceID()));
        }

        // default the inputs
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
        
        flywheel_sim_ =
                new FlywheelSim(
                        LinearSystemId.createFlywheelSystem(
                                motor_type_, wheel_inertia_, gear_ratio_),
                        motor_type_);

        // Setup tunable PIDs
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

            // Set input voltage from motor controller to simulation
            flywheel_sim_.setInput(motors_[0].getSimState().getMotorVoltage());

            // Update simulation by 20ms
            flywheel_sim_.update(0.020);

            // Convert meters to motor rotations
            double motorVelocity =
                    RadiansPerSecond.of(flywheel_sim_.getAngularVelocityRadPerSec() * gear_ratio_)
                            .in(RotationsPerSecond);
            position_ += motorVelocity * 0.020;

            for(int i = 0; i < motors_.length; i++) {
                motors_[i].getSimState().setRawRotorPosition(position_);
                motors_[i].getSimState().setRotorVelocity(motorVelocity);
            }
        }
    }

    @Override
    public void writeOutputs(double timestamp) {
        switch (control_mode_) {
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
     * @return The current velocity of the flywheel in radians per second
     */
    public double getCurrentVelocity() {
        return velocity_;
    }

    /**
     * Sets the target velocity of the flywheel in radians per second
     *
     * @param velocity_rad_per_sec the target velocity in radians per second
     */
    public void setTargetVelocity(double velocity_rad_per_sec) {
        control_mode_ = ControlMode.VELOCITY;
        velocity_target_ = velocity_rad_per_sec;
        velocity_request_.Velocity = Units.radiansToRotations(velocity_rad_per_sec);
    }

    /**
     * Sets the target duty cycle of the flywheel
     *
     * @param duty_cycle the target duty cycle (-1.0 to 1.0)
     */
    public void setTargetDutyCycle(double duty_cycle) {
        control_mode_ = ControlMode.DUTY_CYCLE;
        duty_cycle_target_ = duty_cycle;
        duty_cycle_request_.Output = duty_cycle;
    }

    /**
     * Applies a load torque to the flywheel mechanism for simulation purposes.
     *
     * @param torque_nm The load torque in Newton-meters (Nm). Positive values oppose motion.
     */
    public void applyLoadTorque(double torque_nm) {
        double current_torque = motor_type_.KtNMPerAmp * current_draw_[0];
        current_torque -= torque_nm;

        // Calculate the new angular velocity based on the net torque
        double angular_acceleration = current_torque / wheel_inertia_;
        double new_velocity = velocity_ + angular_acceleration * 0.020; // assuming 20ms timestep

        // Apply the calculated velocity to simulation
        if (IS_SIM) {
            flywheel_sim_.setAngularVelocity(new_velocity);
        }
    }
}
