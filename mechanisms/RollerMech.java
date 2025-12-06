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

    // Simulation
    private final DCMotorSim roller_sim_;

    public RollerMech(FxMotorConfig motor_config) {
        super();

        List<FxMotorConfig> motor_configs = new ArrayList<>();
        motor_configs.add(motor_config);

        ConstructedMotors configured_motors = configMotors(motor_configs, 1.0);

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

        DCMotor motor_type;
        if (motor_config.motor_type == FxMotorType.X60) {
            motor_type = DCMotor.getKrakenX60(1);
        } else {
            throw new IllegalArgumentException("Unsupported motor type for ArmMech");
        }

        roller_sim_ = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, 1.0),
            motor_type);
    }

    protected void configSlot(int slot, SlotConfigs config) {
        if (slot == 0) {
            motor_.getConfigurator().apply(Slot0Configs.from(config));
        } else if (slot == 1) {
            motor_.getConfigurator().apply(Slot1Configs.from(config));
        } else {
            throw new IllegalArgumentException("Slot must be 0, 1, or 2");
        }
    }

    public void setPositionSlot(SlotConfigs config) {
        configSlot(0, config);
    }

    public void setVelocitySlot(SlotConfigs config) {
        configSlot(1, config);
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
