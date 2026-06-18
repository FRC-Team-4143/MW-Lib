package com.marswars.util;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import dev.doglog.DogLog;
import org.wpilib.math.controller.PIDController;

import java.util.function.Consumer;

public class TunablePid {

    /**
     * Creates tunable PID parameters for CTRE Phoenix 6 SlotConfigs.
     * Publishes kP, kI, kD, kS, kV, kA, and kG to NetworkTables for live tuning.
     *
     * @param key The NetworkTables key prefix for the PID parameters
     * @param config_applier Consumer that applies the updated configuration to the motor
     * @param config The initial SlotConfigs to use as default values
     */
    public static void create(
            String key, Consumer<SlotConfigs> config_applier, SlotConfigs config) {
        DogLog.tunable(key + "/kP", config.kP, newP -> config_applier.accept(config.withKP(newP)));
        DogLog.tunable(key + "/kI", config.kI, newI -> config_applier.accept(config.withKI(newI)));
        DogLog.tunable(key + "/kD", config.kD, newD -> config_applier.accept(config.withKD(newD)));
        DogLog.tunable(key + "/kS", config.kS, newS -> config_applier.accept(config.withKS(newS)));
        DogLog.tunable(key + "/kV", config.kV, newV -> config_applier.accept(config.withKV(newV)));
        DogLog.tunable(key + "/kA", config.kA, newA -> config_applier.accept(config.withKA(newA)));
        DogLog.tunable(key + "/kG", config.kG, newG -> config_applier.accept(config.withKG(newG)));
    }

    /**
     * Creates tunable PID parameters for a single WPILib PIDController.
     * Publishes kP, kI, and kD to NetworkTables for live tuning.
     *
     * @param key The NetworkTables key prefix for the PID parameters
     * @param controller The PIDController to tune
     */
    public static void create(String key, PIDController controller) {
        DogLog.tunable(
                key + "/kP", controller.getP(), newP -> controller.setP(newP));
        DogLog.tunable(
                key + "/kI", controller.getI(), newI -> controller.setI(newI));
        DogLog.tunable(
                key + "/kD", controller.getD(), newD -> controller.setD(newD));
    }

    /**
     * Creates tunable PID parameters for multiple synchronized WPILib PIDControllers.
     * All controllers will be updated together when any parameter changes.
     * Publishes kP, kI, and kD to NetworkTables for live tuning.
     *
     * @param key The NetworkTables key prefix for the PID parameters
     * @param controllers Variable number of PIDControllers to tune together
     */
    public static void create(String key, PIDController... controllers) {
        if (controllers.length == 0) {
            return;
        }
        PIDController first = controllers[0];
        DogLog.tunable(
                key + "/kP", first.getP(), newP -> {
                    for (PIDController controller : controllers) {
                        controller.setP(newP);
                    }
                });
        DogLog.tunable(
                key + "/kI", first.getI(), newI -> {
                    for (PIDController controller : controllers) {
                        controller.setI(newI);
                    }
                });
        DogLog.tunable(
                key + "/kD", first.getD(), newD -> {
                    for (PIDController controller : controllers) {
                        controller.setD(newD);
                    }
                });
        }

        /**
         * Creates tunable PID parameters for a Phoenix PIDController (used in swerve).
         * Publishes kP, kI, and kD to NetworkTables for live tuning.
         *
         * @param key The NetworkTables key prefix for the PID parameters
         * @param controller The PhoenixPIDController to tune
         */
        public static void create(String key, PhoenixPIDController controller) {
            DogLog.tunable(
                    key + "/kP", controller.getP(), newP -> controller.setP(newP));
            DogLog.tunable(
                    key + "/kI", controller.getI(), newI -> controller.setI(newI));
            DogLog.tunable(
                    key + "/kD", controller.getD(), newD -> controller.setD(newD));
        }

    private TunablePid() {}
}
