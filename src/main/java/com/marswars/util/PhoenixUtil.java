package com.marswars.util;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class PhoenixUtil {
    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) break;
        }
    }

    /**
     * Converts a boolean invert flag to a Phoenix InvertedValue enum.
     * 
     * @param isInverted true if the motor should be inverted, false otherwise
     * @return the corresponding InvertedValue enum
     */
    public static InvertedValue toInvertedValue(boolean isInverted) {
        return isInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
    }

    public static class TalonFXMotorControllerSim implements SimulatedMotorController {
        private static int instances = 0;
        public final int id;

        private final TalonFXSimState talonFXSimState;

        /**
         * Creates a simulated motor controller for a TalonFX motor.
         * 
         * @param talonFX the TalonFX motor to simulate
         */
        public TalonFXMotorControllerSim(TalonFX talonFX) {
            this.id = instances++;

            this.talonFXSimState = talonFX.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            talonFXSimState.setRawRotorPosition(encoderAngle);
            talonFXSimState.setRotorVelocity(encoderVelocity);
            talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            return talonFXSimState.getMotorVoltageMeasure();
        }
    }

    public static class TalonFXMotorControllerWithRemoteCancoderSim
            extends TalonFXMotorControllerSim {
        private final CANcoderSimState remoteCancoderSimState;

        /**
         * Creates a simulated motor controller for a TalonFX motor with a remote CANcoder.
         * 
         * @param talonFX the TalonFX motor to simulate
         * @param cancoder the CANcoder to use for remote sensing
         */
        public TalonFXMotorControllerWithRemoteCancoderSim(TalonFX talonFX, CANcoder cancoder) {
            super(talonFX);
            this.remoteCancoderSimState = cancoder.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            remoteCancoderSimState.setRawPosition(mechanismAngle);
            remoteCancoderSimState.setVelocity(mechanismVelocity);

            return super.updateControlSignal(
                    mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
        }
    }

    /**
     * Gets an array of timestamps for simulation odometry updates.
     * 
     * @return an array of timestamps covering one simulation period
     */
    public static double[] getSimulationOdometryTimeStamps() {
        final double[] odometryTimeStamps =
                new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimeStamps.length; i++) {
            odometryTimeStamps[i] =
                    Timer.getFPGATimestamp()
                            - 0.02
                            + i * SimulatedArena.getSimulationDt().in(Seconds);
        }

        return odometryTimeStamps;
    }
}
