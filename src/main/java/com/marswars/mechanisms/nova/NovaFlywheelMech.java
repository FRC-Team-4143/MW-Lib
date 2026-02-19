package com.marswars.mechanisms.nova;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig.PIDConfiguration;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import com.marswars.util.NovaMotorConfig;
import com.marswars.util.NovaMotorConfig.NovaMotorType;
import com.marswars.util.TunablePid;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

/**
 * ThriftyNova-based mechanism implementation for flywheels with velocity and duty cycle control.
 * Uses ThriftyNova motor controllers.
 * 
 * @apiNote ThriftyNova controllers do not support:
 *          - Motion Magic/Motion Profile
 *          - Status Signals
 *          - Simulation (sim fields left in place for potential future workarounds)
 */
public class NovaFlywheelMech extends NovaMechBase {

    /** Control modes for the flywheel mechanism */
    protected enum ControlMode {
        VELOCITY,
        DUTY_CYCLE
    }

    private ControlMode control_mode_ = ControlMode.DUTY_CYCLE;

    // Temperature threshold for alerts (Celsius)
    private static final double MOTOR_TEMP_THRESHOLD_C = 80.0;

    // Always assume that we have the leader motor in index 0
    private final ThriftyNova motors_[];

    // Alerts for motor monitoring
    protected final Alert[] motor_temp_alerts_;
    protected final Alert[] motor_disconnected_alerts_;
    
    // Debouncers for connection detection
    protected final Debouncer[] motor_conn_debouncers_;

    // Simulation info (not currently supported by ThriftyNova, but left in place for potential future use)
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
    protected double velocity_feedforward_volts_ = 0.0;
    protected double duty_cycle_target_ = 0;
    protected double[] bus_voltage_;
    protected double[] current_draw_;
    protected double[] motor_temp_c_;

    /**
     * Constructs a new NovaFlywheelMech
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs List of motor configurations
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param wheel_inertia Inertia of the flywheel in kg*m^2 (Simulation only)
     * @param wheel_radius Radius of the flywheel in meters (Simulation only)
     */
    public NovaFlywheelMech(
            String logging_prefix,
            List<NovaMotorConfig> motor_configs,
            double gear_ratio,
            double wheel_inertia,
            double wheel_radius) {
        this(logging_prefix, null, motor_configs, gear_ratio, wheel_inertia, wheel_radius);
    }

    /**
     * Constructs a new NovaFlywheelMech
     *
     * @param logging_prefix String prefix for logging
     * @param mech_name Name of the mechanism
     * @param motor_configs List of motor configurations
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param wheel_inertia Inertia of the flywheel in kg*m^2 (Simulation only)
     * @param wheel_radius Radius of the flywheel in meters (Simulation only)
     */
    public NovaFlywheelMech(
            String logging_prefix,
            String mech_name,
            List<NovaMotorConfig> motor_configs,
            double gear_ratio,
            double wheel_inertia,
            double wheel_radius) {
        super(logging_prefix, mech_name);

        // MW-Lib convention: gear_ratio is motor/mechanism
        double sensor_to_mech_ratio = gear_ratio;
        
        NovaMechBase.ConstructedMotors configured_motors = 
                (NovaMechBase.ConstructedMotors) configMotors(
                motor_configs,
                sensor_to_mech_ratio,
                (cfg) -> {
                    // ThriftyNova doesn't support additional configuration
                    return cfg;
                });
        motors_ = configured_motors.motors;

        this.gear_ratio_ = gear_ratio;
        this.wheel_inertia_ = wheel_inertia;
        this.wheel_radius_ = wheel_radius;

        // default the inputs
        position_ = 0;
        velocity_ = 0;
        bus_voltage_ = new double[motors_.length];
        current_draw_ = new double[motors_.length];
        motor_temp_c_ = new double[motors_.length];

        // Initialize alerts for each motor
        motor_temp_alerts_ = new Alert[motors_.length];
        motor_disconnected_alerts_ = new Alert[motors_.length];
        motor_conn_debouncers_ = new Debouncer[motors_.length];
        for (int i = 0; i < motors_.length; i++) {
            motor_temp_alerts_[i] = new Alert(
                    "High temperature on motor " + i + " in " + getLoggingKey(),
                    AlertType.kWarning);
            motor_disconnected_alerts_[i] = new Alert(
                    "Disconnected motor " + i + " in " + getLoggingKey(),
                    AlertType.kError);
            motor_conn_debouncers_[i] = new Debouncer(0.5);
        }

        ////////////////////////
        /// SIMULATION SETUP ///
        ////////////////////////
        // Note: ThriftyNova does not support simulation at this time
        // Simulation fields are left in place for potential future workarounds

        if (motor_configs.get(0).motor_type == NovaMotorType.VORTEX) {
            motor_type_ = DCMotor.getNeoVortex(motor_configs.size());
        } else if (motor_configs.get(0).motor_type == NovaMotorType.NEO_550) {
            motor_type_ = DCMotor.getNeo550(motor_configs.size());
        } else if (motor_configs.get(0).motor_type == NovaMotorType.PULSAR_775) {
            // Pulsar 775 - use a close approximation (CTRE Minion has similar characteristics)
            motor_type_ = DCMotor.getMinion(motor_configs.size());
        } else {
            throw new IllegalArgumentException("Unsupported motor type");
        }

        // construct the simulation object
        flywheel_sim_ =
                new FlywheelSim(
                        LinearSystemId.createFlywheelSystem(
                                motor_type_, wheel_inertia, gear_ratio),
                        motor_type_);

        // Setup tunable PIDs
        TunablePid.create(
                getLoggingKey() + "VelocityGains",
                (gains) -> configVelocitySlot(gains),
                motor_configs.get(0).config.pid1);
        DogLog.tunable(
                getLoggingKey() + "VelocityGains/Setpoint", 0.0, (val) -> setTargetVelocity(val));
        DogLog.tunable(
                getLoggingKey() + "DutyCycle/Setpoint", 0.0, (val) -> setTargetDutyCycle(val));
    }

    /** {@inheritDoc} */
    @Override
    public void readInputs(double timestamp) {
        // ThriftyNova does not use status signals - direct method calls instead
        
        // Read velocity from leader motor
        // ThriftyNova returns values in rotations per second, so we need to convert to radians per second
        // Velocity is in motor RPS, so divide by gear ratio to get mechanism rad/s
        double motor_velocity_rps = motors_[0].getVelocity();
        
        velocity_ = Units.rotationsToRadians(motor_velocity_rps) / gear_ratio_;
        
        // Read current and temperature from all motors
        for (int i = 0; i < motors_.length; i++) {
            bus_voltage_[i] = motors_[i].getVoltage();
            current_draw_[i] = motors_[i].getSupplyCurrent();
            motor_temp_c_[i] = motors_[i].getTemperature();
            
            // Update temperature alerts for each motor
            motor_temp_alerts_[i].set(motor_temp_c_[i] > MOTOR_TEMP_THRESHOLD_C);
            
            // Update connection alerts using temperature as a proxy (non-zero temp indicates connection)
            // ThriftyNova doesn't have a direct isConnected() method, so we use temperature != 0 as a heuristic
            motor_disconnected_alerts_[i].set(!motor_conn_debouncers_[i].calculate(motor_temp_c_[i] != 0.0));
        }

        // Simulation is not currently supported by ThriftyNova
        // The simulation code below is left in place for potential future use
        if (IS_SIM) {
            // TODO: Implement simulation support if ThriftyLib adds sim capabilities
            // For now, we could potentially use a software simulation approach
            
            // Commented out - not functional without ThriftyNova sim support
            /*
            // Get the voltage the motor controller wants to apply
            double controller_voltage = ... // Would need sim state access
            
            // Calculate the torque required to overcome the load at the motor shaft
            double motor_load_torque = sim_load_torque_nm_ * gear_ratio_;
            double load_current = motor_load_torque / motor_type_.KtNMPerAmp;
            double effective_voltage = controller_voltage - (load_current * motor_type_.rOhms);
            
            // Apply the effective voltage to the simulation
            flywheel_sim_.setInput(effective_voltage);
            flywheel_sim_.update(0.020);
            
            // Reset the load torque after applying it
            sim_load_torque_nm_ = 0.0;

            // Update motor velocity from sim
            double mechanismVelocityRadPerSec = flywheel_sim_.getAngularVelocityRadPerSec();
            double motorVelocityRadPerSec = mechanismVelocityRadPerSec * gear_ratio_;
            double motorVelocity = Units.radiansToRotations(motorVelocityRadPerSec);
            position_ += motorVelocity * 0.020;
            
            // Would need to update ThriftyNova sim state here
            */
        }
    }

    /** {@inheritDoc} */
    @Override
    public void writeOutputs(double timestamp) {
        // ThriftyNova uses direct method calls instead of control request objects
        switch (control_mode_) {
            case VELOCITY:
                // Convert target velocity (rad/s) to motor RPS
                double motor_velocity_rps = Units.radiansToRotations(velocity_target_ * gear_ratio_);
                motors_[0].setVelocity(motor_velocity_rps, velocity_feedforward_volts_);
                break;
            case DUTY_CYCLE:
                motors_[0].setPercent(duty_cycle_target_);
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
        DogLog.log(getLoggingKey() + "control/velocity/target", velocity_target_, RadiansPerSecond);
        DogLog.log(getLoggingKey() + "control/velocity/actual", velocity_, RadiansPerSecond);
        DogLog.log(getLoggingKey() + "control/duty_cycle/target", duty_cycle_target_, Percent);

        // per motor data
        for (int i = 0; i < motors_.length; i++) {
            DogLog.log(getLoggingKey() + "motor" + i + "/bus_voltage", bus_voltage_[i], Volts);
            DogLog.log(getLoggingKey() + "motor" + i + "/current_draw", current_draw_[i], Amps);
            DogLog.log(getLoggingKey() + "motor" + i + "/temp", motor_temp_c_[i], Celsius);
        }
    }

    /**
     * Configures the velocity slot (slot 1) with the given PID config
     *
     * @param config the PID config to apply
     */
    private void configVelocitySlot(PIDConfiguration config) {
        motors_[0].pid1.setP(config.p);
        motors_[0].pid1.setI(config.i);
        motors_[0].pid1.setD(config.d);
        motors_[0].pid1.setFF(config.f);
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
        velocity_feedforward_volts_ = 0.0;
    }

    /**
     * Sets the target velocity of the flywheel with voltage feedforward.
     * This allows additional control output while maintaining velocity.
     *
     * @param velocity_rad_per_sec  the target velocity in radians per second
     * @param feedforward_volts     feedforward voltage to apply (0-12V)
     * 
     * @apiNote ThriftyNova uses voltage feedforward.
     */
    public void setTargetVelocityWithFF(double velocity_rad_per_sec, double feedforward_volts) {
        control_mode_ = ControlMode.VELOCITY;
        velocity_target_ = velocity_rad_per_sec;
        velocity_feedforward_volts_ = feedforward_volts;
    }

    /**
     * Sets the target velocity of the flywheel in radians per second using motion profile velocity control.
     * Since ThriftyNova does not support motion profiling, this falls back to standard velocity control.
     *
     * @param velocity_rad_per_sec the target velocity in radians per second
     * 
     * @apiNote ThriftyNova does not support Motion Magic/Motion Profile - falls back to standard velocity control
     */
    public void setTargetVelocityMotionProfile(double velocity_rad_per_sec) {
        setTargetVelocity(velocity_rad_per_sec);
    }

    /**
     * Sets the target duty cycle of the flywheel
     *
     * @param duty_cycle the target duty cycle (-1.0 to 1.0)
     */
    public void setTargetDutyCycle(double duty_cycle) {
        control_mode_ = ControlMode.DUTY_CYCLE;
        duty_cycle_target_ = duty_cycle;
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
