package frc.mw_lib.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.mw_lib.util.FxMotorConfig;
import frc.mw_lib.util.FxMotorConfig.FxMotorType;
import frc.mw_lib.util.TunablePid;

public class RollerMech extends MechBase {

    protected enum ControlMode {
        POSITION,
        VELOCITY,
        DUTY_CYCLE
    }

    private ControlMode control_mode_ = ControlMode.DUTY_CYCLE;

    // Always assume that we have the leader motor in index 0
    private final TalonFX motor_;
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
    protected double applied_voltage_ = 0;
    protected double current_draw_ = 0;
    protected double motor_temp_c_ = 0;
    protected double bus_voltage_ = 0;

    // System parameters
    private final double gear_ratio_ = 1.0;
    private final double roller_inertia_ = 0.00001;

    // Simulation
    private final DCMotor motor_type_;
    private final DCMotorSim roller_sim_;

    /**
     * Constructs a new RollerMech     *
     * @param logging_prefix String prefix for logging
     * @param motor_config Configuration for the roller motor
     */
    public RollerMech(String logging_prefix, FxMotorConfig motor_config) {
        super(logging_prefix);

        List<FxMotorConfig> motor_configs = new ArrayList<>();
        motor_configs.add(motor_config);

        ConstructedMotors configured_motors = configMotors(motor_configs, gear_ratio_);

        // Store system parameters
        position_request_ = new PositionVoltage(0).withSlot(0);
        velocity_request_ = new VelocityVoltage(0).withSlot(1);
        duty_cycle_request_ = new DutyCycleOut(0);

        // convert the list to an array for easy access
        motor_ = configured_motors.motors[0];
        signals_ = configured_motors.signals;

        // default the inputs
        position_ = 0;
        velocity_ = 0;
        applied_voltage_ = 0;
        current_draw_ = 0;
        motor_temp_c_ = 0;
        bus_voltage_ = 0;

        //////////////////////////
        /// SIMULATION SETUP ///
        //////////////////////////

        if (motor_config.motor_type == FxMotorType.X60) {
            motor_type_ = DCMotor.getKrakenX60(1);
        } else {
            throw new IllegalArgumentException("Unsupported motor type for ArmMech");
        }

        roller_sim_ = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor_type_, roller_inertia_, gear_ratio_),
            motor_type_);

        // Setup tunable PIDs
        TunablePid.create(getLoggingKey() + "PositionGains", this::configPositionSlot, SlotConfigs.from(motor_configs.get(0).config.Slot0));
        DogLog.tunable(getLoggingKey() + "PositionGains/Setpoint", 0.0, (val) -> setTargetPosition(val));
        TunablePid.create(getLoggingKey() + "VelocityGains", this::configVelocitySlot, SlotConfigs.from(motor_configs.get(0).config.Slot1));
        DogLog.tunable(getLoggingKey() + "VelocityGains/Setpoint", 0.0, (val) -> setTargetVelocity(val));
    }

    @Override
    public void readInputs(double timestamp) {
        BaseStatusSignal.refreshAll(signals_);

        // always read the sensor data
        position_ = motor_.getPosition().getValue().in(Radians);
        velocity_ = motor_.getVelocity().getValue().in(RadiansPerSecond);
        applied_voltage_ = motor_.getMotorVoltage().getValueAsDouble();
        current_draw_ = motor_.getSupplyCurrent().getValue().in(Amps);
        motor_temp_c_ = motor_.getDeviceTemp().getValue().in(Celsius);
        bus_voltage_ = motor_.getSupplyVoltage().getValueAsDouble();

        // run the simulation update step here if we are simulating
        if (IS_SIM) {
            // Provide a battery voltage to the TalonFX sim so controller output is
            // meaningful
            motor_.getSimState().setSupplyVoltage(12.0);

            // Set input voltage from motor controller to simulation
            roller_sim_.setInput(motor_.getSimState().getMotorVoltage());

            // Update simulation by 20ms
            roller_sim_.update(0.020);

            // Convert meters to motor rotations
            double motorPosition = Radians.of(roller_sim_.getAngularPositionRad()).in(Rotations);
            double motorVelocity = RadiansPerSecond.of(roller_sim_.getAngularVelocityRadPerSec())
                    .in(RotationsPerSecond);

            motor_.getSimState().setRawRotorPosition(motorPosition);
            motor_.getSimState().setRotorVelocity(motorVelocity);
        }
    }

    @Override
    public void writeOutputs(double timestamp) {
        switch (control_mode_) {
            case POSITION:
                motor_.setControl(position_request_);
                break;
            case VELOCITY:
                motor_.setControl(velocity_request_);
                break;
            case DUTY_CYCLE:
                motor_.setControl(duty_cycle_request_);
                break;
            default:
                throw new IllegalStateException("Unexpected control mode: " + control_mode_);
        }

    }

    public void configPositionSlot(SlotConfigs config) {
        configSlot(0, config);
    }

    public void configVelocitySlot(SlotConfigs config) {
        configSlot(1, config);
    }

    private void configSlot(int slot, SlotConfigs config) {
        if (slot == 0) {
            motor_.getConfigurator().apply(Slot0Configs.from(config));
        } else if (slot == 1) {
            motor_.getConfigurator().apply(Slot1Configs.from(config));
        } else {
            throw new IllegalArgumentException("Slot must be 0, 1, or 2");
        }
    }

    public void setCurrentPosition(double position_rad) {
        motor_.setPosition(Units.radiansToRotations(position_rad));
    }

    public double getCurrentPosition() {
        return position_;
    }

    public double getCurrentVelocity() {
        return velocity_;
    }

    public double getLeaderCurrent() {
        return current_draw_;
    }

    public void setTargetPosition(double position_rad) {
        position_target_ = position_rad;
        control_mode_ = ControlMode.POSITION;
        position_request_.Position = Units.radiansToRotations(position_rad);
    }

    public void setTargetVelocity(double velocity_rad_per_sec) {
        control_mode_ = ControlMode.VELOCITY;
        velocity_target_ = velocity_rad_per_sec;
        velocity_request_.Velocity = Units.radiansToRotations(velocity_rad_per_sec);
    }

    public void setTargetDutyCycle(double duty_cycle) {
        control_mode_ = ControlMode.DUTY_CYCLE;
        duty_cycle_target_ = duty_cycle;
        duty_cycle_request_.Output = duty_cycle;
    }

    /**
     * Applies a load torque to the roller mechanism for simulation purposes.
     * @param torque_nm The load torque in Newton-meters (Nm). Positive values oppose motion.
     */
    public void applyLoadTorque(double torque_nm) {
        double current_torque = motor_type_.KtNMPerAmp * current_draw_;
        current_torque -= torque_nm;

        // Calculate the new angular velocity based on the net torque
        double angular_acceleration = current_torque / roller_inertia_;
        double new_velocity = velocity_ + angular_acceleration * 0.020; // assuming 20ms timestep

        // Apply the calculated velocity to simulation
        if (IS_SIM) {
            roller_sim_.setAngularVelocity(new_velocity);
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

        DogLog.log(getLoggingKey() + "motor/applied_voltage", applied_voltage_);
        DogLog.log(getLoggingKey() + "motor/current_draw", current_draw_);
        DogLog.log(getLoggingKey() + "motor/temp_c", motor_temp_c_);
        DogLog.log(getLoggingKey() + "motor/bus_voltage", bus_voltage_);
    }

}
