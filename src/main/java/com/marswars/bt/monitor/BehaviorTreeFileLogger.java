package com.marswars.bt.monitor;

import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.TreeNode;
import com.marswars.bt.decorator.SubTreeNode;
import com.marswars.bt.xml.BtXmlWriter;
import com.marswars.logging.MwLog;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Writes one XML file per behavior-tree run ({@code <name>_<date>_<time>.btlog.xml}) with every
 * node status transition, so a run can be replayed step by step in an editor. Transitions are
 * buffered in memory while the tree runs and written on a background thread when it ends, so no
 * file I/O happens in the robot loop.
 *
 * <p>Timestamps {@code t} are the tree clock ({@code MwLog.timestampSeconds()} by default), the
 * same time base as the AdvantageKit WPILOG, so a file lines up with the match log. Unlike the
 * logged {@code BehaviorTree/<name>/Status} string, which shows each node's state at the end of a
 * tick, this file keeps every transition, including ones that start and finish inside one tick.
 *
 * <p>Format ({@code format="mwlib-btlog" version="1"}):
 *
 * <pre>{@code
 * <BehaviorTreeLog format="mwlib-btlog" version="1" name="CitrusSynergy" tree_id="CitrusSynergy"
 *                  started="2026-10-10T13:16:31.2-05:00" t_start="34.5500" t_end="99.7700"
 *                  result="INTERRUPTED">
 *   <Parameters>
 *     <Parameter name="middle_wait_msec" type="int" value="3000"/>
 *   </Parameters>
 *   <Nodes>   <!-- depth-first pre-order, same fields as the btlive "tree" message -->
 *     <Node uid="1" parent="" type="Sequence" name="CitrusSynergy" category="Control" path="..."/>
 *   </Nodes>
 *   <Transitions>
 *     <T t="34.5500" uid="1" prev="IDLE" status="RUNNING"/>
 *   </Transitions>
 *   <TreeXml><![CDATA[ ...the XML the tree was built from... ]]></TreeXml>
 * </BehaviorTreeLog>
 * }</pre>
 */
public final class BehaviorTreeFileLogger {
    public static final String FORMAT = "mwlib-btlog";
    public static final int VERSION = 1;
    public static final String EXTENSION = ".btlog.xml";

    private static final AtomicReference<BehaviorTreeFileLogger> ACTIVE = new AtomicReference<>();
    private static final DateTimeFormatter FILE_TIME =
            DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_SSS");

    private final Path dir_;
    private final String suffix_;
    private final ExecutorService writer_ =
            Executors.newSingleThreadExecutor(
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
    }

    public Path directory() {
        return dir_;
    }

    /** Starts recording {@code tree}; call {@link Run#finish} when it stops. */
    public Run start(String name, BehaviorTree tree) {
        return new Run(name, tree);
    }

    /** Completes when the most recent file has been written (with its path). */
    public CompletableFuture<Path> lastWrite() {
        return last_write_;
    }

    /** One recorded run. */
    public final class Run {
        private record Transition(double t, int uid, NodeStatus prev, NodeStatus status) {}

        private final String name_;
        private final BehaviorTree tree_;
        private final ZonedDateTime started_ = ZonedDateTime.now();
        private final double t_start_;
        private final List<Transition> transitions_ = new ArrayList<>();
        private final TreeNode.StatusListener listener_;
        private final Map<String, Object> parameters_;
        private boolean finished_ = false;

        private Run(String name, BehaviorTree tree) {
            name_ = Objects.requireNonNull(name, "name");
            tree_ = Objects.requireNonNull(tree, "tree");
            t_start_ = tree.getClock().getAsDouble();
            parameters_ = tree.getBlackboard().localEntries();
            listener_ = (node, prev, next, t) -> record(node, prev, next, t);
            tree.addStatusListener(listener_);
        }

        private synchronized void record(TreeNode node, NodeStatus prev, NodeStatus next, double t) {
            transitions_.add(new Transition(t, node.getUid(), prev, next));
        }

        /** Stops recording and writes the file in the background. */
        public synchronized CompletableFuture<Path> finish(String result) {
            if (finished_) {
                return last_write_;
            }
            finished_ = true;
            tree_.removeStatusListener(listener_);
            double t_end = tree_.getClock().getAsDouble();
            String xml = render(result, t_end);
            Path file =
                    dir_.resolve(
                            sanitize(name_) + "_" + started_.format(FILE_TIME) + suffix_ + EXTENSION);
            CompletableFuture<Path> write =
                    CompletableFuture.supplyAsync(
                            () -> {
                                try {
                                    Files.createDirectories(dir_);
                                    Files.writeString(file, xml, StandardCharsets.UTF_8);
                                    return file;
                                } catch (IOException e) {
                                    throw new IllegalStateException(
                                            "cannot write behavior tree log " + file, e);
                                }
                            },
                            writer_);
            last_write_ = write;
            return write;
        }

        private String render(String result, double tEnd) {
            StringBuilder sb = new StringBuilder(4096 + transitions_.size() * 64);
            sb.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
            sb.append("<BehaviorTreeLog");
            attr(sb, "format", FORMAT);
            attr(sb, "version", Integer.toString(VERSION));
            attr(sb, "name", name_);
            attr(sb, "tree_id", tree_.getMainTreeId());
            attr(sb, "started", started_.toOffsetDateTime().toString());
            attr(sb, "t_start", time(t_start_));
            attr(sb, "t_end", time(tEnd));
            attr(sb, "result", result == null ? "" : result);
            sb.append(">\n");

            sb.append("  <Parameters>\n");
            for (Map.Entry<String, Object> e : parameters_.entrySet()) {
                Object v = e.getValue();
                if (v instanceof Number || v instanceof Boolean || v instanceof String
                        || v instanceof Enum<?>) {
                    sb.append("    <Parameter");
                    attr(sb, "name", e.getKey());
                    attr(sb, "type", typeName(v));
                    attr(sb, "value", v instanceof Enum<?> en ? en.name() : v.toString());
                    sb.append("/>\n");
                }
            }
            sb.append("  </Parameters>\n");

            sb.append("  <Nodes>\n");
            for (TreeNode node : tree_.getNodes()) {
                sb.append("    <Node");
                attr(sb, "uid", Integer.toString(node.getUid()));
                attr(sb, "parent",
                        tree_.getParent(node).map(p -> Integer.toString(p.getUid())).orElse(""));
                attr(sb, "type", node.getRegistrationId());
                attr(sb, "name", node.getName());
                attr(sb, "category", node.kind().xmlTag());
                attr(sb, "path", node.getPath());
                if (node instanceof SubTreeNode st) {
                    attr(sb, "subtree", st.subtreeId());
                }
                sb.append("/>\n");
            }
            sb.append("  </Nodes>\n");

            sb.append("  <Transitions>\n");
            for (Transition tr : transitions_) {
                sb.append("    <T");
                attr(sb, "t", time(tr.t()));
                attr(sb, "uid", Integer.toString(tr.uid()));
                attr(sb, "prev", tr.prev().name());
                attr(sb, "status", tr.status().name());
                sb.append("/>\n");
            }
            sb.append("  </Transitions>\n");

            tree_.getXml().ifPresent(
                    xml -> sb.append("  <TreeXml><![CDATA[")
                            .append(xml.replace("]]>", "]]]]><![CDATA[>"))
                            .append("]]></TreeXml>\n"));
            sb.append("</BehaviorTreeLog>\n");
            return sb.toString();
        }
    }

    private static String time(double t) {
        return String.format(Locale.ROOT, "%.4f", t);
    }

    private static String typeName(Object v) {
        if (v instanceof Integer || v instanceof Long) {
            return "int";
        }
        if (v instanceof Number) {
            return "double";
        }
        if (v instanceof Boolean) {
            return "bool";
        }
        if (v instanceof Enum<?> en) {
            return en.getDeclaringClass().getSimpleName();
        }
        return "std::string";
    }

    private static void attr(StringBuilder sb, String key, String value) {
        sb.append(' ').append(key).append("=\"").append(BtXmlWriter.escape(value)).append('"');
    }

    private static String sanitize(String name) {
        return name.replaceAll("[^A-Za-z0-9_.-]", "_");
    }
}
