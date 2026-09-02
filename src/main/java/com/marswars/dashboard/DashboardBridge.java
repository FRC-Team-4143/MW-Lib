package com.marswars.dashboard;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.Set;
import java.util.function.IntFunction;

/**
 * Bridges a browser-based interactive dashboard (served off the RoboRIO, driven by NT4 from the
 * browser) to robot code over two NetworkTables tables: one the dashboard writes and the robot
 * reads ("ToRobot"), one the robot writes and the dashboard reads ("ToDashboard").
 *
 * <p>This class only knows about wire-level {@code int}/{@code boolean} channels and the
 * edge-detection/dirty-check plumbing around them -- it has no idea what any channel *means*.
 * Bitfield packing, game rules, and 3D pose visualization are all the consumer's job, layered on
 * top of this class in a per-year robot repo.
 *
 * <p>Not a singleton -- a robot may host zero or one of these; there's no reason to force a single
 * shared instance.
 */
public class DashboardBridge {

    /**
     * @param toRobotTable NT table path the dashboard writes to and this bridge reads from (e.g.
     *     "/ReefControls/ToRobot")
     * @param toDashboardTable NT table path this bridge writes to and the dashboard reads from
     * @param webServerPort port to serve the static dashboard web app on
     * @param webServerDeploySubdir subdirectory of the robot project's deploy directory containing
     *     the dashboard's HTML/CSS/JS
     */
    public record Config(
            String toRobotTable,
            String toDashboardTable,
            int webServerPort,
            String webServerDeploySubdir) {}

    private final Config config_;
    private final Set<DashboardChannel> channels_ = new HashSet<>();
    private final Set<String> channel_names_ = new HashSet<>();

    private final Map<DashboardChannel, IntegerSubscriber> int_subscribers_ = new HashMap<>();
    private final Map<DashboardChannel, BooleanSubscriber> bool_subscribers_ = new HashMap<>();
    private final Map<DashboardChannel, IntegerPublisher> int_publishers_ = new HashMap<>();
    private final Map<DashboardChannel, BooleanPublisher> bool_publishers_ = new HashMap<>();

    // "What do we currently believe this channel's value is" -- fed by both inbound reads
    // (readInputs()) and outbound writes (set()). This is what getInt()/getBool() answer from.
    private final Map<DashboardChannel, Integer> current_int_values_ = new HashMap<>();
    private final Map<DashboardChannel, Boolean> current_bool_values_ = new HashMap<>();

    // "Did a new value arrive from the dashboard this tick" -- cleared and repopulated on every
    // readInputs() call. This is what getIntIfChanged()/getBoolIfChanged() answer from.
    private final Map<DashboardChannel, Integer> pending_int_changes_ = new HashMap<>();
    private final Map<DashboardChannel, Boolean> pending_bool_changes_ = new HashMap<>();

    // "What did we last actually publish to ToDashboard" -- updated only inside set(), and used
    // only to decide whether a set() call needs to touch the wire at all. Deliberately kept
    // separate from current_*_values_: see the class-level "two-cache" note in set() below.
    private final Map<DashboardChannel, Integer> last_published_int_ = new HashMap<>();
    private final Map<DashboardChannel, Boolean> last_published_bool_ = new HashMap<>();

    /** Convenience constructor bound to the default (real robot) NetworkTableInstance. */
    public DashboardBridge(Config config, DashboardChannel... channels) {
        this(NetworkTableInstance.getDefault(), config, List.of(channels));
    }

    /**
     * Primary constructor. Takes an explicit {@link NetworkTableInstance} so tests can pass a
     * scratch instance from {@code NetworkTableInstance.create()} instead of polluting the global
     * one shared with real robot code.
     */
    public DashboardBridge(NetworkTableInstance nt, Config config, List<DashboardChannel> channels) {
        config_ = config;
        NetworkTable to_robot = nt.getTable(config.toRobotTable());
        NetworkTable to_dashboard = nt.getTable(config.toDashboardTable());

        for (DashboardChannel channel : channels) {
            // Duplicate detection is by name alone, not by whole-record equality: two channels
            // sharing a name but differing in type/direction would otherwise slip past a
            // Set<DashboardChannel> check, yet NT itself rejects binding a second topic to an
            // already-claimed name with a different type.
            if (!channel_names_.add(channel.name())) {
                throw new IllegalArgumentException("Duplicate dashboard channel: " + channel.name());
            }
            channels_.add(channel);

            if (channel.direction() == DashboardChannel.Direction.BIDIRECTIONAL) {
                switch (channel.type()) {
                    case INTEGER ->
                            int_subscribers_.put(
                                    channel,
                                    to_robot
                                            .getIntegerTopic(channel.name())
                                            .subscribe(0, PubSubOption.keepDuplicates(true)));
                    case BOOLEAN ->
                            bool_subscribers_.put(
                                    channel,
                                    to_robot
                                            .getBooleanTopic(channel.name())
                                            .subscribe(false, PubSubOption.keepDuplicates(true)));
                }
            }

            switch (channel.type()) {
                case INTEGER -> {
                    int_publishers_.put(
                            channel,
                            to_dashboard
                                    .getIntegerTopic(channel.name())
                                    .publish(PubSubOption.keepDuplicates(true)));
                    current_int_values_.put(channel, 0);
                }
                case BOOLEAN -> {
                    bool_publishers_.put(
                            channel,
                            to_dashboard
                                    .getBooleanTopic(channel.name())
                                    .publish(PubSubOption.keepDuplicates(true)));
                    current_bool_values_.put(channel, false);
                }
            }
        }
    }

    /**
     * Starts serving the static dashboard web app. Call once from production robot code (e.g.
     * during subsystem construction); skip in tests, which don't need the HTTP server.
     */
    public void startWebServer() {
        WebServer.start(
                config_.webServerPort(),
                Paths.get(
                                Filesystem.getDeployDirectory().getAbsolutePath(),
                                config_.webServerDeploySubdir())
                        .toString());
    }

    /**
     * Call once per loop, before any {@code getIntIfChanged}/{@code getBoolIfChanged} calls. Drains
     * every bidirectional channel's inbound queue and records which channels received a new value
     * this tick.
     */
    public void readInputs() {
        pending_int_changes_.clear();
        pending_bool_changes_.clear();

        for (var entry : int_subscribers_.entrySet()) {
            if (entry.getValue().readQueue().length > 0) {
                int value = (int) entry.getValue().get();
                pending_int_changes_.put(entry.getKey(), value);
                current_int_values_.put(entry.getKey(), value);
            }
        }
        for (var entry : bool_subscribers_.entrySet()) {
            if (entry.getValue().readQueue().length > 0) {
                boolean value = entry.getValue().get();
                pending_bool_changes_.put(entry.getKey(), value);
                current_bool_values_.put(entry.getKey(), value);
            }
        }
    }

    /** Did the dashboard send a new value for this bidirectional int channel this tick? */
    public OptionalInt getIntIfChanged(DashboardChannel channel) {
        requireRegistered(channel);
        requireType(channel, DashboardChannel.Type.INTEGER);
        requireBidirectional(channel);
        Integer value = pending_int_changes_.get(channel);
        return value == null ? OptionalInt.empty() : OptionalInt.of(value);
    }

    /** Did the dashboard send a new value for this bidirectional boolean channel this tick? */
    public Optional<Boolean> getBoolIfChanged(DashboardChannel channel) {
        requireRegistered(channel);
        requireType(channel, DashboardChannel.Type.BOOLEAN);
        requireBidirectional(channel);
        return Optional.ofNullable(pending_bool_changes_.get(channel));
    }

    /** Last known value for this channel, whether or not it changed this tick. */
    public int getInt(DashboardChannel channel) {
        requireRegistered(channel);
        requireType(channel, DashboardChannel.Type.INTEGER);
        return current_int_values_.get(channel);
    }

    /** Last known value for this channel, whether or not it changed this tick. */
    public boolean getBool(DashboardChannel channel) {
        requireRegistered(channel);
        requireType(channel, DashboardChannel.Type.BOOLEAN);
        return current_bool_values_.get(channel);
    }

    /**
     * Sets the authoritative value the robot reports to the dashboard for this channel. Safe to
     * call unconditionally every loop -- only writes to NetworkTables when the value actually
     * differs from what was last published, so redundant calls cost nothing on the wire.
     *
     * <p>The dirty-check here compares against {@code last_published_int_}, not {@code
     * current_int_values_}. Those two must stay separate: if the dashboard just sent {@code 2} for
     * this channel, {@code readInputs()} already set {@code current_int_values_} to {@code 2}
     * before consumer code calls {@code set(channel, 2)} to echo it back. Comparing against {@code
     * current_int_values_} would see "2 == 2" and skip the publish -- but the dashboard was never
     * actually told the robot accepted that value; nothing has gone out over the wire yet.
     * Comparing against {@code last_published_int_} instead means the first {@code set()} call
     * after any inbound change always publishes (no prior "last published" entry), and only
     * genuinely repeated calls get suppressed.
     */
    public void set(DashboardChannel channel, int value) {
        requireRegistered(channel);
        requireType(channel, DashboardChannel.Type.INTEGER);
        current_int_values_.put(channel, value);
        Integer last = last_published_int_.get(channel);
        if (last != null && last == value) {
            return;
        }
        int_publishers_.get(channel).set(value);
        last_published_int_.put(channel, value);
    }

    /** See {@link #set(DashboardChannel, int)} -- same dirty-check reasoning applies here. */
    public void set(DashboardChannel channel, boolean value) {
        requireRegistered(channel);
        requireType(channel, DashboardChannel.Type.BOOLEAN);
        current_bool_values_.put(channel, value);
        Boolean last = last_published_bool_.get(channel);
        if (last != null && last == value) {
            return;
        }
        bool_publishers_.get(channel).set(value);
        last_published_bool_.put(channel, value);
    }

    /**
     * Publishes {@code newValues} via {@code publisher} only if it differs from {@code
     * previousValues}. Returns the value to remember as "previous" for the next call.
     *
     * <p>For the "recompute a derived {@code Set<T>} every tick, only call {@code
     * StructArrayPublisher.set()} if it actually changed" idiom -- e.g. broadcasting scored
     * game-piece poses to AdvantageScope without spamming NT on every unchanged tick. The pose
     * math itself is inherently game-specific and stays in consumer code; only this dirty-check
     * shape is generic.
     */
    public static <T> Set<T> publishIfChanged(
            StructArrayPublisher<T> publisher,
            Set<T> previousValues,
            Set<T> newValues,
            IntFunction<T[]> arrayGenerator) {
        if (newValues.equals(previousValues)) {
            return previousValues;
        }
        publisher.set(newValues.toArray(arrayGenerator));
        return newValues;
    }

    private void requireRegistered(DashboardChannel channel) {
        if (!channels_.contains(channel)) {
            throw new IllegalArgumentException(
                    "Channel \"" + channel.name() + "\" was not registered with this DashboardBridge");
        }
    }

    private void requireType(DashboardChannel channel, DashboardChannel.Type expected) {
        if (channel.type() != expected) {
            throw new IllegalArgumentException(
                    "Channel \""
                            + channel.name()
                            + "\" is "
                            + channel.type()
                            + ", not "
                            + expected);
        }
    }

    private void requireBidirectional(DashboardChannel channel) {
        if (channel.direction() != DashboardChannel.Direction.BIDIRECTIONAL) {
            throw new IllegalArgumentException(
                    "Channel \"" + channel.name() + "\" is OUTPUT_ONLY and has no inbound value");
        }
    }
}
