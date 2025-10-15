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
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.mw_lib.mechanisms.FxMotorConfig.FxMotorType;

public class ArmMech extends MechBase {

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
    protected BaseStatusSignal[] signals_;

    // Simulation
    private final SingleJointedArmSim arm_sim_;
    private final double gear_ratio_;

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

    public ArmMech(List<FxMotorConfig> motor_configs, double gear_ratio, double length, double mass_kg, double min_angle, double max_angle){
        super();

        // throw a fit if we don't have any motors
        if (motor_configs == null || motor_configs.size() == 0) {
            throw new IllegalArgumentException("Motor configs is null or empty");
        }

        ArrayList<BaseStatusSignal> all_signals_list = new ArrayList<>();
        motors_ = new TalonFX[motor_configs.size()];
        for (int i = 0; i < motor_configs.size(); i++) {
            FxMotorConfig cfg = motor_configs.get(i);
            if (cfg.canbus_name == null || cfg.canbus_name.isEmpty()) {
                throw new IllegalArgumentException("Motor canbus name is null or empty");
            }

            motors_[i] = new TalonFX(cfg.can_id, cfg.canbus_name);
            ArrayList<BaseStatusSignal> motor_signals = new ArrayList<>();

            // Only apply the configs to the first motor, the rest are followers
            if (i == 0) {
                // Configure the motor for position & velocity control with gravity compensation
                cfg.config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
                cfg.config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
                cfg.config.Slot2.GravityType = GravityTypeValue.Arm_Cosine;

                // also force the gear ratio to be correct
                cfg.config.Feedback.SensorToMechanismRatio = gear_ratio;

                motors_[i].getConfigurator().apply(cfg.config);

                motor_signals.add(motors_[i].getPosition());
                motor_signals.add(motors_[i].getVelocity());
            } else {
                // make the rest of the motors followers
                motors_[i].setControl(new StrictFollower(motors_[0].getDeviceID()));
            }

            motor_signals.add(motors_[i].getMotorVoltage());
            motor_signals.add(motors_[i].getSupplyCurrent());
            motor_signals.add(motors_[i].getDeviceTemp());
            // motor_signals.add(motors_[i].getSupplyVoltage()); // skip refreshing voltage
            // to keep bandwidth low

            // Optimize bus usage to the signals we want
            for (BaseStatusSignal s : motor_signals) {
                s.setUpdateFrequency(50); // 50 Hz update rate
            }
            motors_[i].optimizeBusUtilization();

            // keep a master list of signals for refreshing later
            all_signals_list.addAll(motor_signals);
        }

        position_request_ = new PositionVoltage(0).withSlot(0);
        motion_magic_position_request_ = new MotionMagicVoltage(0).withSlot(0);
        velocity_request_ = new VelocityVoltage(0).withSlot(1);
        duty_cycle_request_ = new DutyCycleOut(0);

        // convert the list to an array for easy access
        signals_ = new BaseStatusSignal[all_signals_list.size()];
        signals_ = all_signals_list.toArray(signals_);

        this.gear_ratio_ = gear_ratio;
        this.use_motion_magic_ = motor_configs.get(0).use_motion_magic;

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
        
        DCMotor motor_type;
        if(motor_configs.get(0).motor_type == FxMotorType.X60){
            motor_type = DCMotor.getKrakenX60(motor_configs.size());
        } else {
            throw new IllegalArgumentException("Unsupported motor type for ArmMech");
        }

        arm_sim_ = new SingleJointedArmSim(
            motor_type, // Motor type
            gear_ratio,
            SingleJointedArmSim.estimateMOI(length, mass_kg),
            length, // Length of the arm (meters)
            min_angle, // Minimum angle (radians)
            max_angle,  // Maximum angle (radians)
            true, // Simulate gravity
            0 // Starting angle (radians)
        );
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
            // Set input voltage from motor controller to simulation
            arm_sim_.setInput(motors_[0].getSimState().getMotorVoltage());

            // Update simulation by 20ms
            arm_sim_.update(0.020);

            // Convert meters to motor rotations
            double motorPosition = Radians.of(arm_sim_.getAngleRads() * gear_ratio_).in(Rotations);
            double motorVelocity = RadiansPerSecond.of(arm_sim_.getVelocityRadPerSec() * gear_ratio_).in(RotationsPerSecond);

            motors_[0].getSimState().setRawRotorPosition(motorPosition);
            motors_[0].getSimState().setRotorVelocity(motorVelocity);
        }
    }

    @Override
    public void writeOutputs(double timestamp) {
        switch(control_mode_){
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

    public void setCurrentPosition(double position_rad){
        motors_[0].setPosition(position_rad);
    }

    public double getCurrentPosition(){
        return position_;
    }

    public double getCurrentVelocity(){
        return velocity_;
    }

    public double getLeaderCurrent() {
        return current_draw_[0];
    }

    public void setTargetPosition(double position_rad){
        position_target_ = position_rad;
        if (use_motion_magic_) {
            control_mode_ = ControlMode.MOTION_MAGIC_POSITION;
            motion_magic_position_request_.Position = position_rad;
        } else {
            control_mode_ = ControlMode.POSITION;
            position_request_.Position = position_rad;
        }
    }

    public void setTargetVelocity(double velocity_rad_per_sec){
        control_mode_ = ControlMode.VELOCITY;
        velocity_target_ = velocity_rad_per_sec;
        velocity_request_.Velocity = velocity_rad_per_sec;
    }

    public void setTargetDutyCycle(double duty_cycle){
        control_mode_ = ControlMode.DUTY_CYCLE;
        duty_cycle_target_ = duty_cycle;
        duty_cycle_request_.Output = duty_cycle;
    }

    @Override
    public void logData() {
        // commands
        DogLog.log(getLoggingKey() + "mode", control_mode_.toString());
        DogLog.log(getLoggingKey() + "position/target", position_target_);
        DogLog.log(getLoggingKey() + "position/actual", position_);
        DogLog.log(getLoggingKey() + "velocity/target", velocity_target_); 
        DogLog.log(getLoggingKey() + "velocity/actual", velocity_);
        DogLog.log(getLoggingKey() + "duty_cycle/target", duty_cycle_target_);

        // per motor data
        for (int i = 0; i < motors_.length; i++) {
            DogLog.log(getLoggingKey() + "motor" + i + "/applied_voltage", applied_voltage_[i]);
            DogLog.log(getLoggingKey() + "motor" + i + "/current_draw", current_draw_[i]);
            DogLog.log(getLoggingKey() + "motor" + i + "/temp_c", motor_temp_c_[i]);
            DogLog.log(getLoggingKey() + "motor" + i + "/bus_voltage", bus_voltage_[i]);
        }
    }
    
}
