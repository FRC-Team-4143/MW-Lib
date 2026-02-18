package com.marswars.mechanisms.nova;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig.PIDConfiguration;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import com.marswars.util.NovaMotorConfig;
import com.marswars.util.NovaMotorConfig.NovaMotorType;
import com.marswars.util.TunablePid;

import java.util.List;

/**
 * ThriftyNova-based mechanism implementation for elevators with position, velocity, and duty cycle control.
 * Uses ThriftyNova motor controllers.
 * 
 * @apiNote ThriftyNova controllers do not support:
 *          - Motion Magic/Motion Profile
 *          - Status Signals
 *          - Simulation (sim fields left in place for potential future workarounds)
 */
public class NovaElevatorMech extends NovaMechBase {

    /** Control modes for the elevator mechanism */
    protected enum ControlMode {
        POSITION,
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

    // Simulation (not currently supported by ThriftyNova, but left in place for potential future use)
    private final ElevatorSim elevator_sim_;
    private final double gear_ratio_;
    private final double drum_radius_;
    private final double position_to_rotations_;
    private final DCMotor motor_type_;
    private double sim_load_torque_nm_ = 0.0; // Load torque at drum shaft for simulation

    // sensor inputs
    protected double position_ = 0;
    protected double position_target_ = 0;
    protected double velocity_ = 0;
    protected double velocity_target_ = 0;
    protected double duty_cycle_target_ = 0;
    protected double feedforward_volts_ = 0.0;
    protected double[] applied_voltage_;
    protected double[] current_draw_;
    protected double[] motor_temp_c_;

    /**
     * Constructs a new NovaElevatorMech (assumes vertical elevator)
     *
     * @param logging_prefix String prefix for logging
     * @param motor_configs List of motor configurations
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param drum_radius Radius of the drum in meters
     * @param carriage_mass_kg Mass of the elevator carriage in kg (Simulation only)
     * @param max_extension Maximum extension of the elevator in meters (Simulation only)
     */
    public NovaElevatorMech(
            String logging_prefix,
            List<NovaMotorConfig> motor_configs,
            double gear_ratio,
            double drum_radius,
            double carriage_mass_kg,
            double max_extension) {
        this(logging_prefix, null, motor_configs, gear_ratio, drum_radius, carriage_mass_kg, max_extension, true);
    }

    /**
     * Constructs a new NovaElevatorMech
     *
     * @param logging_prefix String prefix for logging
     * @param mech_name Name of the mechanism
     * @param motor_configs List of motor configurations
     * @param gear_ratio Gear ratio as motor rotations / mechanism rotations
     * @param drum_radius Radius of the drum in meters
     * @param carriage_mass_kg Mass of the elevator carriage in kg (Simulation only)
     * @param max_extension Maximum extension of the elevator in meters (Simulation only)
     * @param is_vertical Whether the elevator is vertical (for gravity compensation)
     */
    public NovaElevatorMech(
            String logging_prefix,
            String mech_name,
            List<NovaMotorConfig> motor_configs,
            double gear_ratio,
            double drum_radius,
            double carriage_mass_kg,
            double max_extension,
            boolean is_vertical) {
        super(logging_prefix, mech_name);

        // MW-Lib convention: gear_ratio is motor/mechanism
        double sensor_to_mech_ratio = gear_ratio;
        
        NovaMechBase.ConstructedMotors configured_motors = 
                (NovaMechBase.ConstructedMotors) configMotors(
                motor_configs,
                sensor_to_mech_ratio,
                (cfg) -> {
                    // ThriftyNova doesn't support gravity compensation configuration
                    // Future: could implement software gravity compensation if needed
                    return cfg;
                });
        motors_ = configured_motors.motors;

        this.gear_ratio_ = gear_ratio;
        this.drum_radius_ = drum_radius;
        this.position_to_rotations_ = 1 / (2.0 * Math.PI * drum_radius_);

        // default the inputs
        position_ = 0;
        velocity_ = 0;
        applied_voltage_ = new double[motors_.length];
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
        // ThriftyNova returns values in rotations, so we need to convert to meters
        // Position is in motor rotations, divide by gear ratio and convert using position_to_rotations_
        double motor_position_rot = motors_[0].getPosition();
        double motor_velocity_rps = motors_[0].getVelocity();
        
        position_ = (motor_position_rot / gear_ratio_) / position_to_rotations_;
        velocity_ = (motor_velocity_rps / gear_ratio_) / position_to_rotations_;
        
        // Read current and temperature from all motors
        for (int i = 0; i < motors_.length; i++) {
            applied_voltage_[i] = motors_[i].getVoltage();
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
            elevator_sim_.setInput(effective_voltage);
            elevator_sim_.update(0.020);
            
            // Reset the load torque after applying it
            sim_load_torque_nm_ = 0.0;

            // Update motor positions from sim
            double mechanismPositionMeters = elevator_sim_.getPositionMeters();
            double mechanismVelocityMPS = elevator_sim_.getVelocityMetersPerSecond();
            double motorPosition = mechanismPositionMeters * position_to_rotations_ * gear_ratio_;
            double motorVelocity = mechanismVelocityMPS * position_to_rotations_ * gear_ratio_;
            
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
                // Convert target position (meters) to motor rotations
                double motor_position_rot = position_target_ * position_to_rotations_ * gear_ratio_;
                motors_[0].setPosition(motor_position_rot, feedforward_volts_);
                break;
            case VELOCITY:
                // Convert target velocity (m/s) to motor RPS
                double motor_velocity_rps = velocity_target_ * position_to_rotations_ * gear_ratio_;
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
        DogLog.log(getLoggingKey() + "control/position/target", position_target_, "m");
        DogLog.log(getLoggingKey() + "control/position/actual", position_, "m");
        DogLog.log(getLoggingKey() + "control/velocity/target", velocity_target_, "m/s");
        DogLog.log(getLoggingKey() + "control/velocity/actual", velocity_, "m/s");
        DogLog.log(getLoggingKey() + "control/duty_cycle/target", duty_cycle_target_, "%");
        DogLog.log(getLoggingKey() + "control/duty_cycle/actual", applied_voltage_[0] / 12.0, "%");
        DogLog.log(getLoggingKey() + "control/feedforward", feedforward_volts_, "volts");

        // per motor data
        for (int i = 0; i < motors_.length; i++) {
            DogLog.log(getLoggingKey() + "motor" + i + "/applied_voltage", applied_voltage_[i], "volts");
            DogLog.log(getLoggingKey() + "motor" + i + "/current_draw", current_draw_[i], "amps");
            DogLog.log(getLoggingKey() + "motor" + i + "/temp", motor_temp_c_[i], "C");
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
     * Sets the current position of the elevator (for zeroing purposes)
     *
     * @param position_m the position to set in meters
     */
    public void setCurrentPosition(double position_m) {
        // Convert mechanism position (meters) to motor rotations
        double motor_position_rot = position_m * position_to_rotations_ * gear_ratio_;
        motors_[0].setEncoderPosition(motor_position_rot);
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
     * Sets the target position of the elevator in meters using standard position control
     *
     * @param position_m the target position in meters
     * 
     * @apiNote ThriftyNova does not support Motion Magic/Motion Profile
     */
    public void setTargetPosition(double position_m) {
        motors_[0].usePIDSlot(ThriftyNova.PIDSlot.SLOT0); // Use slot 0 for position control
        position_target_ = position_m;
        feedforward_volts_ = 0.0;
        control_mode_ = ControlMode.POSITION;
    }

    /**
     * Sets the target position of the elevator in meters using motion profile control.
     * Since ThriftyNova does not support motion profiling, this falls back to standard position control.
     *
     * @param position_m the target position in meters
     * 
     * @apiNote ThriftyNova does not support Motion Magic/Motion Profile - falls back to standard position control
     */
    public void setTargetPositionMotionProfile(double position_m) {
        setTargetPosition(position_m);
    }

    /**
     * Sets the target position of the elevator with arbitrary feed forward.
     *
     * @param position_m the target position in meters
     * @param feedforward_volts arbitrary feed forward value in volts (0-12V)
     * 
     * @apiNote ThriftyNova does not support Motion Magic/Motion Profile
     */
    public void setTargetPositionWithFF(double position_m, double feedforward_volts) {
        motors_[0].usePIDSlot(ThriftyNova.PIDSlot.SLOT0); // Use slot 0 for position control
        position_target_ = position_m;
        feedforward_volts_ = feedforward_volts;
        control_mode_ = ControlMode.POSITION;
    }

    /**
     * Sets the target position of the elevator with arbitrary feed forward using motion profile control.
     * Since ThriftyNova does not support motion profiling, this falls back to standard position control with feedforward.
     *
     * @param position_m the target position in meters
     * @param feedforward_volts arbitrary feed forward value in volts (0-12V)
     * 
     * @apiNote ThriftyNova does not support Motion Magic/Motion Profile - falls back to standard position control with feedforward
     */
    public void setTargetPositionMotionProfileWithFF(double position_m, double feedforward_volts) {
        setTargetPositionWithFF(position_m, feedforward_volts);
    }

    /**
     * Sets the target velocity of the elevator in meters per second
     *
     * @param velocity_mps the target velocity in meters per second
     */
    public void setTargetVelocity(double velocity_mps) {
        motors_[0].usePIDSlot(ThriftyNova.PIDSlot.SLOT1); // Use slot 1 for velocity control
        control_mode_ = ControlMode.VELOCITY;
        velocity_target_ = velocity_mps;
    }

    /**
     * Sets the target duty cycle of the elevator
     *
     * @param duty_cycle the target duty cycle (-1.0 to 1.0)
     */
    public void setTargetDutyCycle(double duty_cycle) {
        control_mode_ = ControlMode.DUTY_CYCLE;
        duty_cycle_target_ = duty_cycle;
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
}
