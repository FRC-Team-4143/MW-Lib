package com.marswars.bt.monitor;

import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.TreeNode;
import com.marswars.bt.xml.BtcppTreeXml;
import com.marswars.logging.MwLog;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.BufferedOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CopyOnWriteArraySet;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Records each behavior-tree run to a {@code .btlog} file in BehaviorTree.CPP's {@code
 * FileLogger2} format, which Groot2 and the BT editor replay.
 *
 * <p>File layout, all integers little-endian:
 *
 * <ol>
 *   <li>{@code "BTCPP4-FileLogger2"}, then one protocol byte {@code 1}
 *   <li>int32 length, then the tree XML as written by BT.CPP's {@code WriteTreeToXML} with
 *       metadata ({@code _uid}, {@code _fullpath}; see {@link BtcppTreeXml})
 *   <li>uint64 start time, in microseconds since the Unix epoch (wall clock)
 *   <li>one 9-byte record per status transition: 6-byte microseconds since the start, uint16
 *       node uid, uint8 status (0 IDLE, 1 RUNNING, 2 SUCCESS, 3 FAILURE, 4 SKIPPED)
 * </ol>
 *
 * <p>Record times come from the tree clock ({@code MwLog.timestampSeconds()} by default), so they
 * line up with the AdvantageKit log and replay identically. Every transition is kept, including
 * ones that start and finish inside a single tick, which the logged {@code Status} string can't
 * show. The robot thread only queues records. A background thread writes the header when a run
 * starts, appends records every {@value #FLUSH_PERIOD_MS} ms, and closes the file when the run
 * ends, so a run cut short by a power loss still leaves a readable log.
 */
public final class BehaviorTreeFileLogger {
    public static final String EXTENSION = ".btlog";
    public static final String MAGIC = "BTCPP4-FileLogger2";
    public static final byte PROTOCOL = 1;
    public static final long FLUSH_PERIOD_MS = 50;

    private static final AtomicReference<BehaviorTreeFileLogger> ACTIVE = new AtomicReference<>();
    private static final DateTimeFormatter FILE_TIME =
            DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_SSS").withZone(ZoneId.systemDefault());

    private final Path dir_;
    private final String suffix_;
    private final Set<Run> open_runs_ = new CopyOnWriteArraySet<>();
    private final ScheduledExecutorService writer_ =
            Executors.newSingleThreadScheduledExecutor(
                    r -> {
                        Thread t = new Thread(r, "bt-file-logger");
                        t.setDaemon(true);
                        return t;
                    });
    private volatile CompletableFuture<Path> last_write_ = CompletableFuture.completedFuture(null);

    /** Logs every BehaviorTreeCommand run into {@code dir} from now on. */
    public static BehaviorTreeFileLogger enable(Path dir) {
        BehaviorTreeFileLogger logger = new BehaviorTreeFileLogger(dir, "");
        ACTIVE.set(logger);
        return logger;
    }

    /** {@link #enable(Path)} in {@link #defaultDirectory()}; files get "_replay" during replay. */
    public static BehaviorTreeFileLogger enableDefault() {
        BehaviorTreeFileLogger logger =
                new BehaviorTreeFileLogger(defaultDirectory(), MwLog.isReplay() ? "_replay" : "");
        ACTIVE.set(logger);
        return logger;
    }

    public static Optional<BehaviorTreeFileLogger> active() {
        return Optional.ofNullable(ACTIVE.get());
    }

    public static void disable() {
        ACTIVE.set(null);
    }

    /** Next to the AdvantageKit logs: {@code /U/logs/bt} on the robot, {@code logs/bt} in sim. */
    public static Path defaultDirectory() {
        return RobotBase.isReal() ? Path.of("/U/logs/bt") : Path.of("logs", "bt");
    }

    public BehaviorTreeFileLogger(Path dir, String fileSuffix) {
        dir_ = Objects.requireNonNull(dir, "dir");
        suffix_ = fileSuffix == null ? "" : fileSuffix;
        writer_.scheduleAtFixedRate(
                this::flushAll, FLUSH_PERIOD_MS, FLUSH_PERIOD_MS, TimeUnit.MILLISECONDS);
    }

    public Path directory() {
        return dir_;
    }

    /** Starts recording {@code tree}; call {@link Run#finish} when it stops. */
    public Run start(String name, BehaviorTree tree) {
        Run run = new Run(name, tree);
        open_runs_.add(run);
        return run;
    }

    /** Completes with the path of the most recently finished file once it is closed. */
    public CompletableFuture<Path> lastWrite() {
        return last_write_;
    }

    private void flushAll() {
        for (Run run : open_runs_) {
            try {
                run.drain();
            } catch (RuntimeException e) {
                // keep the writer thread alive for the other runs
            }
        }
    }

    /** One recorded run (one file). */
    public final class Run {
        private record Transition(double t, int uid, NodeStatus status) {}

        private final BehaviorTree tree_;
        private final Path file_;
        private final double t_start_;
        private final TreeNode.StatusListener listener_;
        private final ConcurrentLinkedQueue<Transition> queue_ = new ConcurrentLinkedQueue<>();
        private final CompletableFuture<Void> opened_;
        private OutputStream out_ = null; // writer thread only
        private IOException error_ = null; // writer thread only
        private boolean finished_ = false;

        private Run(String name, BehaviorTree tree) {
            tree_ = Objects.requireNonNull(tree, "tree");
            Instant start = Instant.now();
            file_ =
                    dir_.resolve(
                            sanitize(Objects.requireNonNull(name, "name"))
                                    + "_"
                                    + FILE_TIME.format(start)
                                    + suffix_
                                    + EXTENSION);
            t_start_ = tree.getClock().getAsDouble();
            byte[] header = header(BtcppTreeXml.write(tree), start);
            listener_ = (node, prev, next, t) -> queue_.add(new Transition(t, node.getUid(), next));
            tree.addStatusListener(listener_);
            opened_ = CompletableFuture.runAsync(() -> open(header), writer_);
        }

        public Path file() {
            return file_;
        }

        private void open(byte[] header) {
            try {
                Files.createDirectories(dir_);
                out_ = new BufferedOutputStream(Files.newOutputStream(file_));
                out_.write(header);
                out_.flush();
            } catch (IOException e) {
                error_ = e;
            }
        }

        /** Writes queued transitions (writer thread). */
        private void drain() {
            if (out_ == null) {
                return;
            }
            ByteBuffer rec = ByteBuffer.allocate(9).order(ByteOrder.LITTLE_ENDIAN);
            Transition t;
            boolean wrote = false;
            try {
                while ((t = queue_.poll()) != null) {
                    long usec = Math.max(0L, Math.round((t.t() - t_start_) * 1e6));
                    rec.clear();
                    for (int i = 0; i < 6; i++) {
                        rec.put((byte) (usec >>> (8 * i)));
                    }
                    rec.putShort((short) t.uid());
                    rec.put(statusCode(t.status()));
                    out_.write(rec.array(), 0, 9);
                    wrote = true;
                }
                if (wrote) {
                    out_.flush();
                }
            } catch (IOException e) {
                error_ = e;
            }
        }

        /**
         * Stops recording, writes the remaining transitions and closes the file in the background.
         * The result is not part of the BT.CPP format; it is logged as {@code
         * BehaviorTree/<name>/Result} instead.
         */
        public synchronized CompletableFuture<Path> finish(String result) {
            if (finished_) {
                return last_write_;
            }
            finished_ = true;
            tree_.removeStatusListener(listener_);
            CompletableFuture<Path> done =
                    opened_.thenApplyAsync(
                            v -> {
                                drain();
                                open_runs_.remove(this);
                                try {
                                    if (out_ != null) {
                                        out_.close();
                                    }
                                } catch (IOException e) {
                                    error_ = e;
                                }
                                if (error_ != null) {
                                    throw new IllegalStateException(
                                            "cannot write behavior tree log " + file_, error_);
                                }
                                return file_;
                            },
                            writer_);
            last_write_ = done;
            return done;
        }
    }

    /** BT.CPP {@code NodeStatus} numbering. */
    public static byte statusCode(NodeStatus status) {
        return switch (status) {
            case IDLE -> 0;
            case RUNNING -> 1;
            case SUCCESS -> 2;
            case FAILURE -> 3;
            case SKIPPED -> 4;
        };
    }

    /** Magic, protocol, XML length + XML, start time in epoch microseconds. */
    static byte[] header(String xml, Instant start) {
        byte[] magic = MAGIC.getBytes(StandardCharsets.US_ASCII);
        byte[] xml_bytes = xml.getBytes(StandardCharsets.UTF_8);
        ByteBuffer b =
                ByteBuffer.allocate(magic.length + 1 + 4 + xml_bytes.length + 8)
                        .order(ByteOrder.LITTLE_ENDIAN);
        b.put(magic);
        b.put(PROTOCOL);
        b.putInt(xml_bytes.length);
        b.put(xml_bytes);
        b.putLong(start.getEpochSecond() * 1_000_000L + start.getNano() / 1_000L);
        return b.array();
    }

    private static String sanitize(String name) {
        return name.replaceAll("[^A-Za-z0-9_.-]", "_");
    }
}
