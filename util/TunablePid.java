package frc.mw_lib.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import dev.doglog.DogLog;
import java.util.function.Consumer;

public class TunablePid {

  public static void create(
      String key, Consumer<Slot0Configs> config_applier, Slot0Configs config) {
    DogLog.tunable(key + "/kP", config.kP, newP -> config_applier.accept(config.withKP(newP)));
    DogLog.tunable(key + "/kI", config.kI, newI -> config_applier.accept(config.withKI(newI)));
    DogLog.tunable(key + "/kD", config.kD, newD -> config_applier.accept(config.withKD(newD)));
    DogLog.tunable(key + "/kS", config.kS, newS -> config_applier.accept(config.withKS(newS)));
    DogLog.tunable(key + "/kV", config.kV, newV -> config_applier.accept(config.withKV(newV)));
    DogLog.tunable(key + "/kA", config.kA, newA -> config_applier.accept(config.withKA(newA)));
    DogLog.tunable(key + "/kG", config.kG, newG -> config_applier.accept(config.withKG(newG)));
  }

  private TunablePid() {}
}
