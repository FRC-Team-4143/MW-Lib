package com.marswars.util;

import com.ctre.phoenix6.configs.SlotConfigs;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;

import java.util.function.Consumer;

public class TunablePid {

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

    public static void create(String key, PIDController controller) {
        DogLog.tunable(
                key + "/kP", controller.getP(), newP -> controller.setP(newP));
        DogLog.tunable(
                key + "/kI", controller.getI(), newI -> controller.setI(newI));
        DogLog.tunable(
                key + "/kD", controller.getD(), newD -> controller.setD(newD));
    }

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

    private TunablePid() {}
}
