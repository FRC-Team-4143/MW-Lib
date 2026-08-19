package com.marswars.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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

    // Live NT metadata publishers (visible on dashboards, distinct from Logger.recordMetadata
    // which bakes the same info into the log file for replay/AdvantageScope's metadata pane)
    private static final StringPublisher project_name_pub_ =
            NetworkTableInstance.getDefault().getStringTopic("/Metadata/PROJECT_NAME").publish();
    private static final StringPublisher git_sha_pub_ =
            NetworkTableInstance.getDefault().getStringTopic("/Metadata/GIT_SHA").publish();
    private static final StringPublisher git_date_pub_ =
            NetworkTableInstance.getDefault().getStringTopic("/Metadata/GIT_DATE").publish();
    private static final StringPublisher git_branch_pub_ =
            NetworkTableInstance.getDefault().getStringTopic("/Metadata/GIT_BRANCH").publish();
    private static final StringPublisher build_date_pub_ =
            NetworkTableInstance.getDefault().getStringTopic("/Metadata/BUILD_DATE").publish();
    private static final StringPublisher dirty_pub_ =
            NetworkTableInstance.getDefault().getStringTopic("/Metadata/DIRTY").publish();
    private static final StringPublisher mwlib_version_pub_ =
            NetworkTableInstance.getDefault().getStringTopic("/Metadata/MWLIB_VERSION").publish();

    private static final Alert dirty_alert_ =
            new Alert("Dirty git directory, this can lead to unreproducible results", AlertType.kInfo);

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    /** Idempotent. Sets up data receivers and calls Logger.start(). */
    public static synchronized void init(Object buildConstants) {
        if (initialized_) return;
        initialized_ = true;

        recordMetadata(buildConstants);
        recordMwLibVersion();

        if (RobotBase.isSimulation()) {
            String replayPath = System.getenv("AKIT_LOG_PATH");
            if (replayPath != null) {
                Logger.setReplaySource(new WPILOGReader(replayPath));
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(replayPath, "_replay")));
            } else {
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            }
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

    /**
     * Records the version of MW-Lib itself, read from the consuming jar's manifest
     * (set at publish time — see publish.gradle). Null when MW-Lib isn't running from a
     * published jar (e.g. local dev builds), in which case "dev" is recorded instead.
     */
    private static void recordMwLibVersion() {
        String version = MwLog.class.getPackage().getImplementationVersion();
        if (version == null) {
            version = "dev";
        }
        Logger.recordMetadata("MwLibVersion", version);
        mwlib_version_pub_.set(version);
    }

    private static void recordMetadata(Object buildConstants) {
        try {
            Class<?> clazz =
                    (buildConstants instanceof Class<?>)
                            ? (Class<?>) buildConstants
                            : buildConstants.getClass();

            String projectName = (String) clazz.getField("MAVEN_NAME").get(null);
            String gitSha      = (String) clazz.getField("GIT_SHA").get(null);
            String gitDate     = (String) clazz.getField("GIT_DATE").get(null);
            String gitBranch   = (String) clazz.getField("GIT_BRANCH").get(null);
            String buildDate   = (String) clazz.getField("BUILD_DATE").get(null);
            int dirtyFlag      = (int) clazz.getField("DIRTY").get(null);

            // Bake metadata into the log file itself (replay/AdvantageScope's metadata pane)
            Logger.recordMetadata("ProjectName", projectName);
            Logger.recordMetadata("GitSHA",      gitSha);
            Logger.recordMetadata("GitDate",     gitDate);
            Logger.recordMetadata("GitBranch",   gitBranch);
            Logger.recordMetadata("BuildDate",   buildDate);
            Logger.recordMetadata("Dirty",       String.valueOf(dirtyFlag));

            // Publish the same info live to NetworkTables (visible on dashboards while running)
            project_name_pub_.set(projectName);
            git_sha_pub_.set(gitSha.length() > 7 ? gitSha.substring(0, 7) : gitSha);
            git_date_pub_.set(gitDate);
            git_branch_pub_.set(gitBranch);
            build_date_pub_.set(buildDate);
            switch (dirtyFlag) {
                case 0:
                    dirty_pub_.set("All changes committed");
                    dirty_alert_.set(false);
                    break;
                case 1:
                    dirty_pub_.set("Uncommitted changes");
                    dirty_alert_.set(true);
                    break;
                default:
                    dirty_pub_.set("Unknown");
                    break;
            }
        } catch (Exception e) {
            System.err.println("MwLog: failed to record metadata: " + e.getMessage());
        }
    }
}
