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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import com.marswars.util.NovaMotorConfig;
import com.marswars.util.NovaMotorConfig.NovaMotorType;
import com.marswars.util.TunablePid;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

/**
 * ThriftyNova-based mechanism implementation for rollers with position, velocity, and duty cycle control.
 * Uses ThriftyNova motor controllers.
 * 
 * @apiNote ThriftyNova controllers do not support:
 *          - Motion Magic/Motion Profile
 *          - Status Signals
 *          - Simulation (sim fields left in place for potential future workarounds)
 */
public class NovaRollerMech extends NovaMechBase {

    /** Control modes for the roller mechanism */
    protected enum ControlMode {
        POSITION,
        VELOCITY,
        DUTY_CYCLE
    }

    private ControlMode control_mode_ = ControlMode.DUTY_CYCLE;

    // Temperature threshold for alerts (Celsius)
    private static final double MOTOR_TEMP_THRESHOLD_C = 65.0;

    // Always assume that we have the leader motor in index 0
    private final ThriftyNova motors_[];

    // Alerts for motor monitoring
    protected final Alert[] motor_temp_alerts_;
    protected final Alert[] motor_disconnected_alerts_;
    
    // Debouncers for connection detection
    protected final Debouncer[] motor_conn_debouncers_;

    // System parameters
    private final double gear_ratio_;
    private final double roller_inertia_;

    // Simulation (not currently supported by ThriftyNova, but left in place for potential future use)
    private final DCMotor motor_type_;
    private final DCMotorSim roller_sim_;
    private double sim_load_torque_nm_ = 0.0; // Load torque at roller shaft for simulation

    // sensor inputs
    protected double position_ = 0;
    protected double position_target_ = 0;
    protected double velocity_ = 0;
    protected double velocity_target_ = 0;
    protected double duty_cycle_target_ = 0;
    protected double feedforward_volts_ = 0.0;
    protected double[] bus_voltage_;
    protected double[] current_draw_;
    protected double[] motor_temp_c_;

    /**
     * Constructs a new NovaRollerMech with a default inertia value.
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs Configuration for the roller motor
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     */
    public NovaRollerMech(String logging_prefix, List<NovaMotorConfig> motor_configs, double gear_ratio) {
        this(logging_prefix, null, motor_configs, gear_ratio, 0.00001);
    }

    /**
     * Constructs a new NovaRollerMech
     *
     * @param logging_prefix String prefix for logging
     * @param mech_name Name of the mechanism
     * @param motor_configs Configuration for the roller motor
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param roller_inertia Inertia of the roller in kg*m^2 (Simulation only)
     */
    public NovaRollerMech(
            String logging_prefix,
            String mech_name,
            List<NovaMotorConfig> motor_configs,
            double gear_ratio,
            double roller_inertia) {
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
        this.roller_inertia_ = roller_inertia;

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
        roller_sim_ =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(motor_type_, roller_inertia, gear_ratio),
                        motor_type_);

        // Setup tunable PIDs
        TunablePid.create(
                getLoggingKey() + "PositionGains",
                (gains) -> configPositionSlot(gains),
                motor_configs.get(0).config.pid0);
        DogLog.tunable(
                getLoggingKey() + "PositionGains/Setpoint", 0.0, (val) -> setTargetPosition(val));
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
        
        // Read position and velocity from leader motor
        // ThriftyNova returns values in rotations, so we need to convert to radians
        // Position is in motor rotations, so divide by gear ratio to get mechanism radians
        double motor_position_rot = motors_[0].getPosition();
        double motor_velocity_rps = motors_[0].getVelocity();
        
        position_ = Units.rotationsToRadians(motor_position_rot) / gear_ratio_;
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
            roller_sim_.setInput(effective_voltage);
            roller_sim_.update(0.020);
            
            // Reset the load torque after applying it
            sim_load_torque_nm_ = 0.0;

            // Update motor positions from sim
            double mechanismPositionRad = roller_sim_.getAngularPositionRad();
            double mechanismVelocityRadPerSec = roller_sim_.getAngularVelocityRadPerSec();
            double motorPositionRad = mechanismPositionRad * gear_ratio_;
            double motorVelocityRadPerSec = mechanismVelocityRadPerSec * gear_ratio_;
            double motorPosition = Units.radiansToRotations(motorPositionRad);
            double motorVelocity = Units.radiansToRotations(motorVelocityRadPerSec);
            
            // Would need to update ThriftyNova sim state here
            */
        }
    }

    /** {@inheritDoc} */
    @Override
    public void writeOutputs(double timestamp) {
        // ThriftyNova uses direct method calls instead of control request objects
        switch (control_mode_) {
            case POSITION:
                // Convert target position (radians) to motor rotations
                double motor_position_rot = Units.radiansToRotations(position_target_ * gear_ratio_);
                motors_[0].setPosition(motor_position_rot, feedforward_volts_);
                break;
            case VELOCITY:
                // Convert target velocity (rad/s) to motor RPS
                double motor_velocity_rps = Units.radiansToRotations(velocity_target_ * gear_ratio_);
                motors_[0].setVelocity(motor_velocity_rps);
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
        DogLog.log(getLoggingKey() + "control/position/target", position_target_, Radians);
        DogLog.log(getLoggingKey() + "control/position/actual", position_, Radians);
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
     * Configures the position slot (slot 0) with the given PID config
     *
     * @param config the PID config to apply
     */
    private void configPositionSlot(PIDConfiguration config) {
        motors_[0].pid0.setP(config.p);
        motors_[0].pid0.setI(config.i);
        motors_[0].pid0.setD(config.d);
        motors_[0].pid0.setFF(config.f);
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
     * Sets the current position of the roller mechanism (for resetting encoders, etc.)
     *
     * @param position_rad the position in radians
     */
    public void setCurrentPosition(double position_rad) {
        double motor_position_rot = Units.radiansToRotations(position_rad * gear_ratio_);
        motors_[0].setEncoderPosition(motor_position_rot);
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
     * Sets the target position of the roller mechanism in radians using standard position control
     *
     * @param position_rad the target position in radians
     * 
     * @apiNote ThriftyNova does not support Motion Magic/Motion Profile
     */
    public void setTargetPosition(double position_rad) {
        motors_[0].usePIDSlot(ThriftyNova.PIDSlot.SLOT0); // Use slot 0 for position control
        position_target_ = position_rad;
        feedforward_volts_ = 0.0;
        control_mode_ = ControlMode.POSITION;
    }

    /**
     * Sets the target position of the roller mechanism with arbitrary feed forward.
     *
     * @param position_rad the target position in radians
     * @param feedforward_volts arbitrary feed forward value in volts (0-12V)
     * 
     * @apiNote ThriftyNova does not support Motion Magic/Motion Profile
     */
    public void setTargetPositionWithFF(double position_rad, double feedforward_volts) {
        motors_[0].usePIDSlot(ThriftyNova.PIDSlot.SLOT0); // Use slot 0 for position control
        position_target_ = position_rad;
        feedforward_volts_ = feedforward_volts;
        control_mode_ = ControlMode.POSITION;
    }

    /**
     * Sets the target position of the roller mechanism in radians using motion profile control.
     * Since ThriftyNova does not support motion profiling, this falls back to standard position control.
     *
     * @param position_rad the target position in radians
     * 
     * @apiNote ThriftyNova does not support Motion Magic/Motion Profile - falls back to standard position control
     */
    public void setTargetPositionMotionProfile(double position_rad) {
        setTargetPosition(position_rad);
    }

    /**
     * Sets the target position of the roller mechanism with arbitrary feed forward using motion profile control.
     * Since ThriftyNova does not support motion profiling, this falls back to standard position control with feedforward.
     *
     * @param position_rad the target position in radians
     * @param feedforward_volts arbitrary feed forward value in volts (0-12V)
     * 
     * @apiNote ThriftyNova does not support Motion Magic/Motion Profile - falls back to standard position control with feedforward
     */
    public void setTargetPositionMotionProfileWithFF(double position_rad, double feedforward_volts) {
        setTargetPositionWithFF(position_rad, feedforward_volts);
    }

    /**
     * Sets the target velocity of the roller mechanism in radians per second using standard velocity control
     *
     * @param velocity_rad_per_sec the target velocity in radians per second
     */
    public void setTargetVelocity(double velocity_rad_per_sec) {
        motors_[0].usePIDSlot(ThriftyNova.PIDSlot.SLOT1); // Use slot 1 for velocity control
        control_mode_ = ControlMode.VELOCITY;
        velocity_target_ = velocity_rad_per_sec;
    }

    /**
     * Sets the target velocity of the roller mechanism in radians per second using motion profile velocity control.
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
     * Sets the target duty cycle of the roller mechanism
     *
     * @param duty_cycle the target duty cycle (-1.0 to 1.0)
     */
    public void setTargetDutyCycle(double duty_cycle) {
        control_mode_ = ControlMode.DUTY_CYCLE;
        duty_cycle_target_ = duty_cycle;
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
