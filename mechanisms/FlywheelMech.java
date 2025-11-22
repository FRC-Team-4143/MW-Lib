package frc.mw_lib.mechanisms;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelMech extends MechBase{

    // control modes for the flywheel mechanism
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

    protected final FlywheelSim flywheel_sim_;
    protected final double gear_ratio_;
    protected final double wheel_inertia_;
    protected final double wheel_radius_;

    public FlywheelMech(List<FxMotorConfig> motor_configs, double gear_ratio, double wheel_inertia, double wheel_radius){
        super();

        // Create control requests
        this.velocity_request_ = new VelocityVoltage(0).withSlot(0);
        this.duty_cycle_request_ = new DutyCycleOut(0);

        ConstructedMotors configured_motors = configMotors(motor_configs, gear_ratio, (cfg) -> {
            // Configure the motor for position & velocity control with gravity compensation
            cfg.config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
            cfg.config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
            cfg.config.Slot2.GravityType = GravityTypeValue.Arm_Cosine;

            return cfg;
        });
        motors_ = configured_motors.motors;
        signals_ = configured_motors.signals;

        // set the system constants
        this.gear_ratio_ = gear_ratio;
        this.wheel_inertia_ = wheel_inertia;
        this.wheel_radius_ = wheel_radius;

        // configure the simulation
        this.flywheel_sim_ = null; // TODO: implement flywheel sim
    }

    @Override
    public void readInputs(double timestamp) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'readInputs'");
    }

    @Override
    public void writeOutputs(double timestamp) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'writeOutputs'");
    }

    @Override
    public void logData() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logData'");
    }
    
}
