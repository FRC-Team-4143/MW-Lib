package com.marswars.auto;

import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.Blackboard;
import com.marswars.bt.core.BtException;
import com.marswars.bt.core.ParameterStore;
import com.marswars.bt.core.TreeNode;
import com.marswars.bt.core.TreeSpec;
import com.marswars.bt.monitor.BehaviorTreeCommand;
import com.marswars.bt.monitor.BehaviorTreeMonitor;
import com.marswars.bt.monitor.BehaviorTreeRunner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * An auto defined by a BehaviorTree.CPP v4 XML file (typically {@code
 * deploy/autos/trees/<Name>.xml}). It is an {@link AutoRoutine}, not a WPILib command: a {@link
 * BehaviorTreeRunner} ticks the tree straight from the robot loop.
 *
 * <p>This class only wires small pieces together:
 *
 * <ul>
 *   <li>{@link TreeLoad}: parse and check the XML and load the paths it names ({@link
 *       TrajectoryNames}), on a background thread;
 *   <li>{@link LoggedXml}: log the text, so replay rebuilds the same tree;
 *   <li>{@link TreeParameters}: the auto's tunable values, {@code /Tuning/Autos/<auto>/<port>};
 *   <li>{@link AutoTrajectories}: the loaded, alliance-flipped paths;
 *   <li>{@link BehaviorTreeRunner}: one run of the tree.
 * </ul>
 *
 * <p>Nodes get trajectories through the root blackboard entry {@code @trajectories}:
 *
 * <pre>{@code
 * ChoreoTrajectory traj = BehaviorTreeAuto.trajectories(node).apply(node.getString("trajectory"));
 * }</pre>
 *
 * <p>A file that fails to load keeps the last good version (if any), raises an {@link Alert} and
 * logs {@code BehaviorTree/<name>/Error}.
 */
public class BehaviorTreeAuto implements AutoRoutine {
    /** Root blackboard key of the {@code Function<String, ChoreoTrajectory>} resolver. */
    public static final String TRAJECTORIES_KEY = "trajectories";
    /** Root blackboard key holding this auto's name. */
    public static final String AUTO_NAME_KEY = "auto";

    /** Where load messages go; replaceable so unit tests don't start a DataLog file. */
    static Consumer<String> message_log_ = DataLogManager::log;

    private final String name_;
    private final BehaviorTreeFactory factory_;
    private final Supplier<String> xml_source_;
    private final Path base_dir_;
    private final TreeParameters parameters_;
    private final AutoTrajectories paths_ = new AutoTrajectories();
    private final LoggedXml logged_xml_;
    private final BehaviorTreeMonitor monitor_;
    private final BehaviorTreeRunner runner_;

    private TreeLoad loaded_ = null; // the last good load
    private String error_ = "";
    private Alert alert_ = null;

    /** Auto named after the file ({@code CitrusSynergy.xml} gives {@code CitrusSynergy}). */
    public BehaviorTreeAuto(BehaviorTreeFactory factory, Path xmlFile) {
        this(factory, xmlFile, TreeParameters.defaultStore());
    }

    /** Like {@link #BehaviorTreeAuto(BehaviorTreeFactory, Path)} with an explicit parameter store. */
    public BehaviorTreeAuto(BehaviorTreeFactory factory, Path xmlFile, ParameterStore parameters) {
        this(
                factory,
                AutoFiles.autoName(xmlFile),
                () -> AutoFiles.read(xmlFile),
                xmlFile.toAbsolutePath().getParent(),
                parameters);
    }

    /**
     * @param name chooser/log name
     * @param xmlSource supplies the XML text each time the auto reloads
     * @param baseDir directory {@code <include>} paths resolve against (may be null)
     */
    public BehaviorTreeAuto(
            BehaviorTreeFactory factory, String name, Supplier<String> xmlSource, Path baseDir) {
        this(factory, name, xmlSource, baseDir, TreeParameters.defaultStore());
    }

    /**
     * @param parameters where tree parameters live between runs (dashboard by default)
     */
    public BehaviorTreeAuto(
            BehaviorTreeFactory factory,
            String name,
            Supplier<String> xmlSource,
            Path baseDir,
            ParameterStore parameters) {
        name_ = Objects.requireNonNull(name, "name");
        factory_ = Objects.requireNonNull(factory, "factory");
        xml_source_ = Objects.requireNonNull(xmlSource, "xmlSource");
        base_dir_ = baseDir;
        parameters_ = new TreeParameters(name, parameters);
        logged_xml_ = new LoggedXml(BehaviorTreeMonitor.PREFIX + name);
        monitor_ = new BehaviorTreeMonitor(name);
        runner_ = new BehaviorTreeRunner(name, this::buildTree, monitor_);
        reload();
    }

    /** One auto per auto file in {@code dir} (see {@link AutoFiles#listAutos}), by file name. */
    public static List<BehaviorTreeAuto> loadAll(BehaviorTreeFactory factory, Path dir) {
        return loadAll(factory, dir, TreeParameters.defaultStore());
    }

    public static List<BehaviorTreeAuto> loadAll(
            BehaviorTreeFactory factory, Path dir, ParameterStore parameters) {
        List<BehaviorTreeAuto> autos = new ArrayList<>();
        for (Path file : AutoFiles.listAutos(dir)) {
            autos.add(new BehaviorTreeAuto(factory, file, parameters));
        }
        return autos;
    }

    /** The {@code @trajectories} resolver from a node's blackboard. */
    @SuppressWarnings("unchecked")
    public static Function<String, ChoreoTrajectory> trajectories(TreeNode node) {
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

    @Override
    public String getName() {
        return name_;
    }

    /** Store key of a tree parameter: {@code Autos/<auto>/<port>} (dashboard {@code /Tuning/...}). */
    public String parameterKey(String port) {
        return parameters_.key(port);
    }

    // ------------------------------------------------------------------ loading

    /**
     * Re-reads and checks the XML now, on this thread, and registers its trajectory names (the
     * paths themselves load in {@link #prepare}). Returns true when the new XML was accepted.
     */
    public boolean reload() {
        return use(load(null), null);
    }

    /** Reads, checks and loads the paths off the robot thread; the returned step puts it in use. */
    @Override
    public Runnable prepare(boolean is_red_alliance) {
        TreeLoad load = load(is_red_alliance);
        return () -> use(load, is_red_alliance);
    }

    private TreeLoad load(Boolean is_red_alliance) {
        String xml;
        try {
            xml = xml_source_.get();
        } catch (RuntimeException e) {
            return TreeLoad.failed(null, "cannot read XML: " + e.getMessage());
        }
        return TreeLoad.of(xml, base_dir_, factory_, newRoot(), is_red_alliance);
    }

    /** Puts a load in use on the robot thread, or keeps the last good one if it failed. */
    private synchronized boolean use(TreeLoad load, Boolean is_red_alliance) {
        String xml = logged_xml_.process(load.xml());
        if (!Objects.equals(xml, load.xml())) { // log replay: run what the robot ran
            load = TreeLoad.of(xml, base_dir_, factory_, newRoot(), is_red_alliance);
        }
        if (!load.ok()) {
            setError(load.error());
            return false;
        }
        loaded_ = load;
        parameters_.publish(load.spec());
        paths_.use(load.trajectoryNames(), load.trajectories());
        for (String warning : load.spec().warnings()) {
            message_log_.accept("[BehaviorTree] " + name_ + ": " + warning);
        }
        setError("");
        return true;
    }

    /** A run's root blackboard: the trajectory resolver and the auto's name. */
    private Blackboard newRoot() {
        Blackboard root = Blackboard.createRoot();
        Function<String, ChoreoTrajectory> resolver = name -> paths_.get(name).get();
        root.set(TRAJECTORIES_KEY, resolver);
        root.set(AUTO_NAME_KEY, name_);
        return root;
    }

    /** Builds a fresh tree for one run from the last good load, with the current tunable values. */
    public synchronized BehaviorTree buildTree() {
        if (loaded_ == null) {
            throw new BtException(error_.isEmpty() ? "no tree loaded" : error_);
        }
        Blackboard root = newRoot();
        parameters_.seed(loaded_.spec(), root);
        return factory_.createTree(loaded_.spec(), root, loaded_.xml());
    }

    private void setError(String error) {
        error_ = error == null ? "" : error;
        monitor_.publishError(error_);
        if (!error_.isEmpty()) {
            message_log_.accept("[BehaviorTree] " + name_ + ": " + error_);
            if (alert_ == null) {
                alert_ = new Alert("", AlertType.kError);
            }
        }
        if (alert_ != null) {
            alert_.setText("BT auto " + name_ + ": " + error_);
            alert_.set(!error_.isEmpty());
        }
    }

    public synchronized TreeSpec getSpec() {
        return loaded_ == null ? null : loaded_.spec();
    }

    public synchronized String getXml() {
        return loaded_ == null ? null : loaded_.xml();
    }

    /** Last load error, or empty when the current XML loaded cleanly. */
    public synchronized String getError() {
        return error_;
    }

    public synchronized boolean isLoaded() {
        return loaded_ != null;
    }

    /** Names of the trajectories this auto loads, in load order. */
    public List<String> getTrajectoryNames() {
        return paths_.names();
    }

    @Override
    public Pose2d getStartPose() {
        return paths_.startPose();
    }

    @Override
    public Pose2d[] getPath() {
        return paths_.path();
    }

    // ------------------------------------------------------------------ running

    @Override
    public void start() {
        runner_.start();
    }

    @Override
    public void periodic() {
        runner_.tick();
    }

    @Override
    public void stop(boolean interrupted) {
        runner_.stop(interrupted);
    }

    @Override
    public boolean isRunning() {
        return runner_.isRunning();
    }

    /** This auto as a WPILib command, for using the tree inside a command composition. */
    public BehaviorTreeCommand asCommand() {
        return new BehaviorTreeCommand(name_, this::buildTree, monitor_);
    }
}
