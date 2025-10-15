package frc.mw_lib.mechanisms;

import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorMech extends MechBase {

    // Always assume that we have the leader motor in index 0
    private final TalonFX motors[];
    private final PositionVoltage positionRequest;
    private final VelocityVoltage velocityRequest;

    // Simulation
    private final ElevatorSim elevatorSim;

    public ElevatorMech(List<FxMotorConfig> motor_configs, double gearRatio, double drumRadius, double carriageMassKg,
            double maxExtension) {
        this(motor_configs, gearRatio, drumRadius, carriageMassKg, maxExtension, false);
    }

    public ElevatorMech(List<FxMotorConfig> motor_configs, double gearRatio, double drumRadius, double carriageMassKg,
            double maxExtension, boolean is_vertical) {
        super();

        // throw a fit if we don't have any motors
        if (motor_configs == null || motor_configs.size() == 0) {
            throw new IllegalArgumentException("Motor configs is null or empty");
        }

        motors = new TalonFX[motor_configs.size()];
        for (int i = 0; i < motor_configs.size(); i++) {
            FxMotorConfig cfg = motor_configs.get(i);
            if (cfg.canbus_name == null || cfg.canbus_name.isEmpty()) {
                throw new IllegalArgumentException("Motor canbus name is null or empty");
            }

            motors[i] = new TalonFX(cfg.can_id, cfg.canbus_name);

            // Only apply the configs to the first motor, the rest are followers
            if (i == 0) {
                // Configure the motor for position & velocity control with gravity compensation
                if (is_vertical) {
                    cfg.config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
                    cfg.config.Slot1.GravityType = GravityTypeValue.Elevator_Static;
                    cfg.config.Slot2.GravityType = GravityTypeValue.Elevator_Static;
                }

                // also force the gear ratio to be correct
                cfg.config.Feedback.SensorToMechanismRatio = gearRatio;

                motors[i].getConfigurator().apply(cfg.config);
            }
        }

        positionRequest = new PositionVoltage(0).withSlot(0);
        velocityRequest = new VelocityVoltage(0).withSlot(1);

        // construct the simulation object
        elevatorSim = new ElevatorSim(
                DCMotor.getKrakenX60(1), // Motor type
                gearRatio,
                carriageMassKg, // Carriage mass (kg)
                drumRadius, // Drum radius (m)
                0,
                maxExtension, // Max height (m)
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
        motors[0].setPosition(position);
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
