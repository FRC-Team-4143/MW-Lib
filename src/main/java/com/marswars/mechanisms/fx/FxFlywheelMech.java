package com.marswars.mechanisms.fx;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;
import com.marswars.util.TunablePid;
import java.util.List;

/**
 * TalonFX-based mechanism implementation for flywheels with velocity and duty cycle control.
 */
public class FxFlywheelMech extends FxMechBase {

    /** Control modes for the flywheel mechanism */
    protected enum ControlMode {
        MOTION_MAGIC_VELOCITY,
        VELOCITY,
        DUTY_CYCLE
    }

    private ControlMode control_mode_ = ControlMode.DUTY_CYCLE;

    // Temperature threshold for alerts (Celsius)
    private static final double MOTOR_TEMP_THRESHOLD_C = 80.0;

    // Always assume that we have the leader motor in index 0
    private final TalonFX motors_[];

    // control and status
    private final VelocityVoltage velocity_request_;
    protected final MotionMagicVelocityVoltage motion_magic_velocity_request_;
    private final DutyCycleOut duty_cycle_request_;
    protected final BaseStatusSignal[] signals_;

    // Alerts for motor monitoring
    protected final Alert[] motor_disconnected_alerts_;
    protected final Alert[] motor_temp_alerts_;
    protected final Debouncer[] motor_conn_debouncers_;

    // Simulation info
    protected final FlywheelSim flywheel_sim_;
    protected final double gear_ratio_;
    protected final double wheel_inertia_;
    protected final double wheel_radius_;
    private final DCMotor motor_type_;
    private double sim_load_torque_nm_ = 0.0; // Load torque at flywheel shaft for simulation

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
     * Constructs a new FxFlywheelMech
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs List of motor configurations
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param wheel_inertia Inertia of the flywheel in kg*m^2 (Simulation only)
     * @param wheel_radius Radius of the flywheel in meters (Simulation only)
     */
    public FxFlywheelMech(
            String logging_prefix,
            List<FxMotorConfig> motor_configs,
            double gear_ratio,
            double wheel_inertia,
            double wheel_radius) {
        this(logging_prefix, null, motor_configs, gear_ratio, wheel_inertia, wheel_radius);
    }

    /**
     * Constructs a new FxFlywheelMech
     *
     * @param logging_prefix String prefix for logging
     * @param mech_name Name of the mechanism
     * @param motor_configs List of motor configurations
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param wheel_inertia Inertia of the flywheel in kg*m^2 (Simulation only)
     * @param wheel_radius Radius of the flywheel in meters (Simulation only)
     */
    public FxFlywheelMech(
            String logging_prefix,
            String mech_name,
            List<FxMotorConfig> motor_configs,
            double gear_ratio,
            double wheel_inertia,
            double wheel_radius) {
        super(logging_prefix, mech_name);

        // Create control requests
        this.velocity_request_ = new VelocityVoltage(0).withSlot(1);
        this.motion_magic_velocity_request_ = new MotionMagicVelocityVoltage(0).withSlot(1);
        this.duty_cycle_request_ = new DutyCycleOut(0);

        // MW-Lib convention: gear_ratio is motor/mechanism
        // Phoenix convention: SensorToMechanismRatio = sensor/mechanism = motor/mechanism
        // These are the same, so we can use gear_ratio directly
        double sensor_to_mech_ratio = gear_ratio;
        
        FxMechBase.ConstructedFxMotors configured_motors = 
                (FxMechBase.ConstructedFxMotors) configMotors(motor_configs, sensor_to_mech_ratio);
        motors_ = configured_motors.motors;
        signals_ = configured_motors.signals;

        // set the system constants
        this.gear_ratio_ = gear_ratio;
        this.wheel_inertia_ = wheel_inertia;
        this.wheel_radius_ = wheel_radius;

        // default the inputs
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

    /** {@inheritDoc} */
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
            
            // Update alerts for each motor
            motor_disconnected_alerts_[i].set(!motor_conn_debouncers_[i].calculate(motors_[i].isConnected()));
            motor_temp_alerts_[i].set(motor_temp_c_[i] > MOTOR_TEMP_THRESHOLD_C);
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
            // (load torque at flywheel * gear ratio = load torque at motor)
            double motor_load_torque = sim_load_torque_nm_ * gear_ratio_;
            
            // Calculate the current needed to produce this load torque
            double load_current = motor_load_torque / motor_type_.KtNMPerAmp;
            
            // The voltage actually seen by the motor after the load consumes some current
            // is reduced by the voltage drop across the resistance due to load current
            double effective_voltage = controller_voltage - (load_current * motor_type_.rOhms);
            
            // Apply the effective voltage to the simulation
            flywheel_sim_.setInput(effective_voltage);

            // Update simulation by 20ms
            flywheel_sim_.update(0.020);

            // Reset the load torque after applying it (impulse load)
            // This must be called again each cycle for sustained load
            sim_load_torque_nm_ = 0.0;

            // Convert mechanism velocity to motor velocity
            // gear_ratio_ = motor/mechanism, so motor = mechanism * gear_ratio_
            double mechanismVelocityRadPerSec = flywheel_sim_.getAngularVelocityRadPerSec();
            double motorVelocityRadPerSec = mechanismVelocityRadPerSec * gear_ratio_;
            
            double motorVelocity = RadiansPerSecond.of(motorVelocityRadPerSec).in(RotationsPerSecond);
            position_ += motorVelocity * 0.020;

            for(int i = 0; i < motors_.length; i++) {
                motors_[i].getSimState().setRawRotorPosition(position_);
                motors_[i].getSimState().setRotorVelocity(motorVelocity);
                
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
            case MOTION_MAGIC_VELOCITY:
                motors_[0].setControl(motion_magic_velocity_request_);
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
        DogLog.log(getLoggingKey() + "control/velocity/target", velocity_target_, "rad/s");
        DogLog.log(getLoggingKey() + "control/velocity/actual", velocity_, "rad/s");
        DogLog.log(getLoggingKey() + "control/duty_cycle/target", duty_cycle_target_, "%");
        DogLog.log(getLoggingKey() + "control/duty_cycle/actual", applied_voltage_[0] / 12.0, "%");

        // per motor data
        for (int i = 0; i < motors_.length; i++) {
            DogLog.log(getLoggingKey() + "motor" + i + "/applied_voltage", applied_voltage_[i], "volts");
            DogLog.log(getLoggingKey() + "motor" + i + "/current_draw", current_draw_[i], "amps");
            DogLog.log(getLoggingKey() + "motor" + i + "/temp", motor_temp_c_[i], "C");
            DogLog.log(getLoggingKey() + "motor" + i + "/bus_voltage", bus_voltage_[i], "volts");
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
     * Sets the target velocity of the flywheel in radians per second using standard velocity control
     *
     * @param velocity_rad_per_sec the target velocity in radians per second
     */
    public void setTargetVelocity(double velocity_rad_per_sec) {
        control_mode_ = ControlMode.VELOCITY;
        velocity_target_ = velocity_rad_per_sec;
        velocity_request_.Velocity = Units.radiansToRotations(velocity_rad_per_sec);
    }

    /**
     * Sets the target velocity of the flywheel in radians per second using motion magic velocity control
     *
     * @param velocity_rad_per_sec the target velocity in radians per second
     */
    public void setTargetVelocityMotionMagic(double velocity_rad_per_sec) {
        control_mode_ = ControlMode.MOTION_MAGIC_VELOCITY;
        velocity_target_ = velocity_rad_per_sec;
        motion_magic_velocity_request_.Velocity = Units.radiansToRotations(velocity_rad_per_sec);
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
     * This method should be called during the simulation update cycle to apply
     * external loads (like friction, compression forces, etc.) to the mechanism.
     *
     * @param torque_nm The load torque in Newton-meters (Nm) at the flywheel output shaft.
     *                  Positive values oppose motion in the positive direction.
     */
    public void applyLoadTorque(double torque_nm) {
        sim_load_torque_nm_ = torque_nm;
    }
}
