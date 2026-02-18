package com.marswars.util;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig.PIDConfiguration;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;

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
     * Creates tunable PID parameters for Thrifty Nova PIDConfiguration.
     * Publishes kP, kI, kD, and kF to NetworkTables for live tuning.
     *
     * @param key The NetworkTables key prefix for the PID parameters
     * @param config_applier Consumer that applies the updated configuration to the motor
     * @param config The initial PIDConfiguration to use as default values
     */
    public static void create(
            String key, Consumer<PIDConfiguration> config_applier, PIDConfiguration config) {
        DogLog.tunable(key + "/kP", config.p, newP -> {
            PIDConfiguration newConfig = new PIDConfiguration();
            newConfig.p = newP;
            newConfig.i = config.i;
            newConfig.d = config.d;
            newConfig.f = config.f;
            config_applier.accept(newConfig);
        });
        DogLog.tunable(key + "/kI", config.i, newI -> {
            PIDConfiguration newConfig = new PIDConfiguration();
            newConfig.p = config.p;
            newConfig.i = newI;
            newConfig.d = config.d;
            newConfig.f = config.f;
            config_applier.accept(newConfig);
        });
        DogLog.tunable(key + "/kD", config.d, newD -> {
            PIDConfiguration newConfig = new PIDConfiguration();
            newConfig.p = config.p;
            newConfig.i = config.i;
            newConfig.d = newD;
            newConfig.f = config.f;
            config_applier.accept(newConfig);
        });
        DogLog.tunable(key + "/kF", config.f, newF -> {
            PIDConfiguration newConfig = new PIDConfiguration();
            newConfig.p = config.p;
            newConfig.i = config.i;
            newConfig.d = config.d;
            newConfig.f = newF;
            config_applier.accept(newConfig);
        });
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
