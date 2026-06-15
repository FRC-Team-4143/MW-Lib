package com.marswars.logging;

import edu.wpi.first.units.Unit;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Logging facade for MW-Lib. Wraps AdvantageKit's Logger with DogLog-compatible call signatures
 * so all log/tunable call sites in both repos are a mechanical DogLog→MwLog find/replace.
 *
 * <p>Call MwLog.init(BuildConstants.class) once (SubsystemManager does this). All other methods
 * are safe to call before init returns (Logger queues them). Call MwLog.periodic() at the top of
 * each control loop to poll tunables and fire onChange consumers.
 */
public final class MwLog {

    private MwLog() {}

    private static boolean initialized_ = false;

    // Tunable registry — LoggedNetworkNumber.periodic() is auto-driven by the Logger;
    // we only need to detect value changes and invoke the consumer.
    private record TunableEntry(LoggedNetworkNumber number, double[] last, DoubleConsumer consumer) {}
    private static final List<TunableEntry> tunables_ = new ArrayList<>();

    // time()/timeEnd() registry
    private static final Map<String, Double> timing_starts_ = new HashMap<>();

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    /** Idempotent. Sets up data receivers and calls Logger.start(). */
    public static synchronized void init(Object buildConstants) {
        if (initialized_) return;
        initialized_ = true;

        recordMetadata(buildConstants);

        if (isReplay()) {
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(
                    new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
        } else if (RobotBase.isSimulation()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();
    }

    /** True when running a log replay (AKIT_LOG_PATH env var is set). */
    public static boolean isReplay() {
        return Logger.hasReplaySource();
    }

    /** Deterministic loop timestamp in seconds (sourced from the log during replay). */
    public static double timestampSeconds() {
        return Logger.getTimestamp() / 1e6;
    }

    /**
     * Call once at the top of each control loop. Polls tunables and fires onChange consumers when
     * a value has changed since last loop.
     */
    public static void periodic() {
        for (TunableEntry entry : tunables_) {
            double current = entry.number().get();
            if (current != entry.last()[0]) {
                entry.last()[0] = current;
                entry.consumer().accept(current);
            }
        }
    }

    // -------------------------------------------------------------------------
    // Tunables
    // -------------------------------------------------------------------------

    /**
     * Registers a live-tunable double backed by a NetworkTables entry under /Tuning/.
     * The consumer is fired immediately with defaultValue, then on every subsequent change.
     * Tunable changes are captured in the log and replay identically.
     */
    public static void tunable(String key, double defaultValue, DoubleConsumer onChange) {
        LoggedNetworkNumber number = new LoggedNetworkNumber("/Tuning/" + key, defaultValue);
        // Use a single-element array so the lambda can capture a mutable reference
        double[] last = {defaultValue};
        tunables_.add(new TunableEntry(number, last, onChange));
        onChange.accept(defaultValue);
    }

    // -------------------------------------------------------------------------
    // Timing
    // -------------------------------------------------------------------------

    /** Starts a named timer. Call timeEnd(key) to record elapsed seconds. */
    public static void time(String key) {
        timing_starts_.put(key, timestampSeconds());
    }

    /** Records elapsed seconds since time(key) was called. */
    public static void timeEnd(String key) {
        Double start = timing_starts_.remove(key);
        if (start != null) {
            Logger.recordOutput(key, timestampSeconds() - start);
        }
    }

    // -------------------------------------------------------------------------
    // log overloads (mirrors the DogLog.log signatures in use across both repos)
    // -------------------------------------------------------------------------

    public static void log(String key, double value)                    { Logger.recordOutput(key, value); }
    public static void log(String key, double value, Unit unit)         { Logger.recordOutput(key, value, unit); }
    public static void log(String key, double[] value)                  { Logger.recordOutput(key, value); }
    public static void log(String key, double[] value, Unit unit)       { Logger.recordOutput(key, value); }
    public static void log(String key, boolean value)                   { Logger.recordOutput(key, value); }
    public static void log(String key, long value)                      { Logger.recordOutput(key, value); }
    public static void log(String key, String value)                    { Logger.recordOutput(key, value); }
    public static void log(String key, String[] value)                  { Logger.recordOutput(key, value); }
    public static <E extends Enum<E>> void log(String key, E value)     { Logger.recordOutput(key, value); }

    /** Logs any single WPI struct (Pose2d, Pose3d, ChassisSpeeds, Rotation2d, …). */
    public static <T extends StructSerializable> void log(String key, T value) {
        Logger.recordOutput(key, value);
    }

    /** Logs any WPI struct array (SwerveModuleState[], SwerveModulePosition[], Pose2d[], …). */
    public static <T extends StructSerializable> void log(String key, T[] value) {
        Logger.recordOutput(key, value);
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    private static void recordMetadata(Object buildConstants) {
        try {
            Class<?> clazz =
                    (buildConstants instanceof Class<?>)
                            ? (Class<?>) buildConstants
                            : buildConstants.getClass();
            Logger.recordMetadata("ProjectName", (String) clazz.getField("MAVEN_NAME").get(null));
            Logger.recordMetadata("GitSHA",      (String) clazz.getField("GIT_SHA").get(null));
            Logger.recordMetadata("GitDate",     (String) clazz.getField("GIT_DATE").get(null));
            Logger.recordMetadata("GitBranch",   (String) clazz.getField("GIT_BRANCH").get(null));
            Logger.recordMetadata("BuildDate",   (String) clazz.getField("BUILD_DATE").get(null));
            Logger.recordMetadata("Dirty",       String.valueOf(clazz.getField("DIRTY").get(null)));
        } catch (Exception e) {
            System.err.println("MwLog: failed to record metadata: " + e.getMessage());
        }
    }
}
