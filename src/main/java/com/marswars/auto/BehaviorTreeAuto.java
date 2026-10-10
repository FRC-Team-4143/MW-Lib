package com.marswars.auto;

import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.Blackboard;
import com.marswars.bt.core.BtException;
import com.marswars.bt.core.NodeModel;
import com.marswars.bt.core.NodeSpec;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.PortType;
import com.marswars.bt.core.PortValues;
import com.marswars.bt.core.TreeSpec;
import com.marswars.bt.monitor.BehaviorTreeCommand;
import com.marswars.bt.monitor.BehaviorTreeMonitor;
import com.marswars.bt.xml.BtXmlParser;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * An {@link Auto} defined by a BehaviorTree.CPP v4 XML file (typically {@code
 * deploy/autos/<Name>.xml}).
 *
 * <p>Every TRAJECTORY-typed port with a literal value (including values passed into SubTrees) is
 * registered with {@link #loadTrajectory}, in document order, so the chooser preview, start pose
 * and alliance flipping work exactly like hand-written autos. Nodes resolve trajectory names
 * through the root blackboard entry {@code @trajectories}, a {@code Function<String,
 * ChoreoTrajectory>} backed by this auto's alliance-flipped cache:
 *
 * <pre>{@code
 * Function<String, ChoreoTrajectory> trajectories = BehaviorTreeAuto.trajectories(node);
 * ChoreoTrajectory traj = trajectories.apply(node.getString("trajectory"));
 * }</pre>
 *
 * <p>The XML is re-read whenever the auto is (re)selected or the alliance changes ({@link
 * #cacheTrajetories}), so copying a new file to the robot and re-selecting is enough to pick up
 * edits. The text is processed as an AdvantageKit input, so log replay rebuilds the same tree. A
 * file that fails to load keeps the last good version (if any), raises an {@link Alert}, and logs
 * {@code BehaviorTree/<name>/Error}.
 */
public class BehaviorTreeAuto extends Auto {
    /** Root blackboard key of the {@code Function<String, ChoreoTrajectory>} resolver. */
    public static final String TRAJECTORIES_KEY = "trajectories";
    /** Root blackboard key holding this auto's name. */
    public static final String AUTO_NAME_KEY = "auto";

    /** Where load messages go; replaceable so unit tests don't start a DataLog file. */
    static Consumer<String> message_log_ = DataLogManager::log;

    private final BehaviorTreeFactory factory_;
    private final Supplier<String> xml_source_;
    private final Path base_dir_;
    private final BehaviorTreeMonitor monitor_;
    private final XmlInputs inputs_ = new XmlInputs();
    private Alert alert_ = null;

    private TreeSpec spec_ = null;
    private String xml_ = null;
    private String error_ = "";

    /** Auto named after the file stem ({@code CitrusSynergyBt.xml} gives {@code CitrusSynergyBt}). */
    public BehaviorTreeAuto(BehaviorTreeFactory factory, Path xmlFile) {
        this(factory, stem(xmlFile), () -> readFile(xmlFile), xmlFile.toAbsolutePath().getParent());
    }

    /**
     * @param name chooser/log name
     * @param xmlSource supplies the XML text each time the auto reloads
     * @param baseDir directory {@code <include>} paths resolve against (may be null)
     */
    public BehaviorTreeAuto(
            BehaviorTreeFactory factory, String name, Supplier<String> xmlSource, Path baseDir) {
        factory_ = Objects.requireNonNull(factory, "factory");
        xml_source_ = Objects.requireNonNull(xmlSource, "xmlSource");
        base_dir_ = baseDir;
        setName(name);
        monitor_ = new BehaviorTreeMonitor(name);
        reload();
        addCommands(new BehaviorTreeCommand(name, this::buildTree, monitor_));
    }

    /** One auto per {@code *.xml} in {@code dir}, sorted by file name (empty if the dir is missing). */
    public static List<BehaviorTreeAuto> loadAll(BehaviorTreeFactory factory, Path dir) {
        List<BehaviorTreeAuto> autos = new ArrayList<>();
        for (Path file : listXml(dir)) {
            autos.add(new BehaviorTreeAuto(factory, file));
        }
        return autos;
    }

    /** Sorted {@code *.xml} files directly in {@code dir}. */
    public static List<Path> listXml(Path dir) {
        if (!Files.isDirectory(dir)) {
            return List.of();
        }
        try (Stream<Path> files = Files.list(dir)) {
            return files.filter(p -> p.getFileName().toString().endsWith(".xml"))
                    .filter(Files::isRegularFile)
                    .sorted()
                    .toList();
        } catch (IOException e) {
            throw new BtException("cannot list " + dir + ": " + e.getMessage(), e);
        }
    }

    /** The {@code @trajectories} resolver from a node's blackboard. */
    @SuppressWarnings("unchecked")
    public static Function<String, ChoreoTrajectory> trajectories(
            com.marswars.bt.core.TreeNode node) {
        Object resolver = node.blackboard().get("@" + TRAJECTORIES_KEY);
        if (!(resolver instanceof Function<?, ?>)) {
            throw new BtException(
                    "["
                            + node.getPath()
                            + "] no trajectory resolver on the blackboard; is this tree running"
                            + " inside a BehaviorTreeAuto?");
        }
        return (Function<String, ChoreoTrajectory>) resolver;
    }

    // ------------------------------------------------------------------ loading

    /**
     * Re-reads and validates the XML, and re-registers its trajectories. Keeps the last good tree
     * on failure. Returns true when the new XML was accepted.
     */
    public synchronized boolean reload() {
        String xml;
        try {
            inputs_.xml = xml_source_.get();
        } catch (RuntimeException e) {
            inputs_.xml = null;
            setError("cannot read XML: " + e.getMessage());
        }
        Logger.processInputs(BehaviorTreeMonitor.PREFIX + getName(), inputs_);
        xml = inputs_.xml;
        if (xml == null) {
            return false;
        }

        TreeSpec spec;
        try {
            spec = BtXmlParser.parse(xml, base_dir_);
            // Instantiate once to validate IDs/ports/arity now rather than at enable time.
            factory_.createTree(spec, Blackboard.createRoot(), xml);
        } catch (RuntimeException e) {
            setError(e.getMessage());
            return false;
        }

        spec_ = spec;
        xml_ = xml;
        trajectories_.clear();
        for (String name : discoverTrajectoryNames(spec, factory_)) {
            loadTrajectory(name);
        }
        for (String warning : spec.warnings()) {
            message_log_.accept("[BehaviorTree] " + getName() + ": " + warning);
        }
        setError("");
        monitor_.publishXml(xml);
        return true;
    }

    /** Reloads the XML, then loads (and alliance-flips) its trajectories. */
    @Override
    public void cacheTrajetories(boolean is_red_alliance) {
        reload();
        super.cacheTrajetories(is_red_alliance);
    }

    /** Builds a fresh tree for one run; the root blackboard carries the trajectory resolver. */
    public synchronized BehaviorTree buildTree() {
        if (spec_ == null) {
            throw new BtException(error_.isEmpty() ? "no tree loaded" : error_);
        }
        Blackboard root = Blackboard.createRoot();
        Function<String, ChoreoTrajectory> resolver = name -> getTrajectory(name).get();
        root.set(TRAJECTORIES_KEY, resolver);
        root.set(AUTO_NAME_KEY, getName());
        return factory_.createTree(spec_, root, xml_);
    }

    public synchronized TreeSpec getSpec() {
        return spec_;
    }

    public synchronized String getXml() {
        return xml_;
    }

    /** Last load error, or empty when the current XML loaded cleanly. */
    public synchronized String getError() {
        return error_;
    }

    public synchronized boolean isLoaded() {
        return spec_ != null;
    }

    /** Names of the trajectories this auto loads, in load order. */
    public List<String> getTrajectoryNames() {
        synchronized (trajectories_) {
            return List.copyOf(trajectories_.keySet());
        }
    }

    private void setError(String error) {
        error_ = error == null ? "" : error;
        monitor_.publishError(error_);
        if (!error_.isEmpty()) {
            message_log_.accept("[BehaviorTree] " + getName() + ": " + error_);
        }
        if (alert_ == null && !error_.isEmpty()) {
            alert_ = new Alert("", AlertType.kError);
        }
        if (alert_ != null) {
            alert_.setText("BT auto " + getName() + ": " + error_);
            alert_.set(!error_.isEmpty());
        }
    }

    // ------------------------------------------------------------------ trajectory discovery

    /**
     * Literal values of TRAJECTORY ports in the main tree, in document order and without
     * duplicates. SubTrees are expanded in place; a {@code {key}} inside a SubTree resolves to a
     * literal its {@code <SubTree>} element passes for that key.
     */
    public static List<String> discoverTrajectoryNames(
            TreeSpec spec, BehaviorTreeFactory factory) {
        Set<String> names = new LinkedHashSet<>();
        if (spec.mainTreeId() != null && spec.trees().containsKey(spec.mainTreeId())) {
            walk(spec.mainTree(), spec, factory, Map.of(), new ArrayList<>(), names);
        }
        return List.copyOf(names);
    }

    private static void walk(
            NodeSpec node,
            TreeSpec spec,
            BehaviorTreeFactory factory,
            Map<String, String> literals,
            List<String> stack,
            Set<String> out) {
        if (node.isSubTree()) {
            NodeSpec target = spec.trees().get(node.subtreeId());
            if (target == null || stack.contains(node.subtreeId())) {
                return;
            }
            Map<String, String> inner = new HashMap<>();
            for (Map.Entry<String, String> attr : node.attributes().entrySet()) {
                String value = resolve(attr.getValue(), attr.getKey(), literals);
                if (value != null && !attr.getKey().startsWith("_")) {
                    inner.put(attr.getKey(), value);
                }
            }
            stack.add(node.subtreeId());
            walk(target, spec, factory, inner, stack, out);
            stack.remove(stack.size() - 1);
            return;
        }
        NodeModel model = factory.getNodeModel(node.id()).orElse(null);
        if (model != null) {
            for (PortInfo port : model.ports()) {
                if (port.type() != PortType.TRAJECTORY) {
                    continue;
                }
                String raw = node.attributes().getOrDefault(port.name(), port.defaultValue());
                String value = resolve(raw, port.name(), literals);
                if (value != null && !value.isBlank()) {
                    out.add(value.trim());
                }
            }
        }
        for (NodeSpec child : node.children()) {
            walk(child, spec, factory, literals, stack, out);
        }
    }

    /** Literal value of {@code raw}, following {@code {key}} into {@code literals}; else null. */
    private static String resolve(String raw, String portName, Map<String, String> literals) {
        if (raw == null) {
            return null;
        }
        if (PortValues.isPointer(raw)) {
            return literals.get(PortValues.pointerKey(raw, portName));
        }
        return raw;
    }

    // ------------------------------------------------------------------ helpers

    private static String stem(Path file) {
        String n = file.getFileName().toString();
        int dot = n.lastIndexOf('.');
        return dot > 0 ? n.substring(0, dot) : n;
    }

    private static String readFile(Path file) {
        try {
            return Files.readString(file, StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new BtException("cannot read " + file + ": " + e.getMessage(), e);
        }
    }

    /** XML text as an AdvantageKit input so replay rebuilds the tree that actually ran. */
    private static final class XmlInputs implements LoggableInputs {
        String xml = null;

        @Override
        public void toLog(LogTable table) {
            table.put("XmlInput", xml == null ? "" : xml);
        }

        @Override
        public void fromLog(LogTable table) {
            String logged = table.get("XmlInput", "");
            xml = logged.isEmpty() ? null : logged;
        }
    }
}
