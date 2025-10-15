package frc.mw_lib.mechanisms;

import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.mw_lib.mechanisms.FxMotorConfig.FxMotorType;

public class ElevatorMech extends MechBase {

    // Always assume that we have the leader motor in index 0
    private final TalonFX motors_[];
    private final PositionVoltage position_request_;
    private final VelocityVoltage velocity_request_;

    // Simulation
    private final ElevatorSim elevator_sim_;

    public ElevatorMech(List<FxMotorConfig> motor_configs, double gear_ratio, double drum_radius, double carriage_mass_kg,
    double max_extension, double rigging_ratio) {
        this(motor_configs, gear_ratio, drum_radius, carriage_mass_kg, max_extension, rigging_ratio, false);
    }

    public ElevatorMech(List<FxMotorConfig> motor_configs, double gear_ratio, double drum_radius, double carriage_mass_kg,
            double max_extension, double rigging_ratio, boolean is_vertical) {
        super();

        // throw a fit if we don't have any motors
        if (motor_configs == null || motor_configs.size() == 0) {
            throw new IllegalArgumentException("Motor configs is null or empty");
        }

        motors_ = new TalonFX[motor_configs.size()];
        for (int i = 0; i < motor_configs.size(); i++) {
            FxMotorConfig cfg = motor_configs.get(i);
            if (cfg.canbus_name == null || cfg.canbus_name.isEmpty()) {
                throw new IllegalArgumentException("Motor canbus name is null or empty");
            }

            motors_[i] = new TalonFX(cfg.can_id, cfg.canbus_name);

            // Only apply the configs to the first motor, the rest are followers
            if (i == 0) {
                // Configure the motor for position & velocity control with gravity compensation
                if (is_vertical) {
                    cfg.config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
                    cfg.config.Slot1.GravityType = GravityTypeValue.Elevator_Static;
                    cfg.config.Slot2.GravityType = GravityTypeValue.Elevator_Static;
                }

                // also force the gear ratio to be correct
                cfg.config.Feedback.SensorToMechanismRatio = gear_ratio;

                motors_[i].getConfigurator().apply(cfg.config);
            } else {
                // make the rest of the motors followers
                motors_[i].setControl(new StrictFollower(motors_[0].getDeviceID()));
            }
        }

        position_request_ = new PositionVoltage(0).withSlot(0);
        velocity_request_ = new VelocityVoltage(0).withSlot(1);

        DCMotor motor_type;
        if(motor_configs.get(0).motor_type == FxMotorType.X60){
            motor_type = DCMotor.getKrakenX60(motor_configs.size());
        } else {
            throw new IllegalArgumentException("Unsupported motor type");
            // motor_type = DCMotor.getKr(motor_configs.size());
        } 

        // construct the simulation object
        elevator_sim_ = new ElevatorSim(
                motor_type, // Motor type
                gear_ratio,
                carriage_mass_kg, // Carriage mass (kg)
                drum_radius, // Drum radius (m)
                0,
                max_extension, // Max height (m)
                is_vertical, // Simulate gravity
                0 // Starting height (m)
        );
    }

    @Override
    public void readInputs(double timestamp) {
        if (IS_SIM) {

        } else {

        }
    }

    @Override
    public void writeOutputs(double timestamp) {
        if (IS_SIM) {

        } else {

        }
    }

    public void setCurrentPosition(double position, double feedforward) {
        motors_[0].setPosition(position);
    }

    public double getPosition() {
        return 0;
    }

    public double getVelocity() {
        return 0;
    }

    @Override
    public void logData() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logData'");
    }

}
