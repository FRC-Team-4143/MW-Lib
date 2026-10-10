package com.marswars.bt.core;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.marswars.bt.action.AlwaysFailureNode;
import com.marswars.bt.action.AlwaysSuccessNode;
import com.marswars.bt.action.RunCommandNode;
import com.marswars.bt.action.SetBlackboardNode;
import com.marswars.bt.action.SleepNode;
import com.marswars.bt.action.UnsetBlackboardNode;
import com.marswars.bt.control.FallbackNode;
import com.marswars.bt.control.IfThenElseNode;
import com.marswars.bt.control.ParallelAllNode;
import com.marswars.bt.control.ParallelDeadlineNode;
import com.marswars.bt.control.ParallelNode;
import com.marswars.bt.control.ReactiveFallbackNode;
import com.marswars.bt.control.ReactiveSequenceNode;
import com.marswars.bt.control.SequenceNode;
import com.marswars.bt.control.SequenceWithMemoryNode;
import com.marswars.bt.control.WhileDoElseNode;
import com.marswars.bt.decorator.DelayNode;
import com.marswars.bt.decorator.ForceFailureNode;
import com.marswars.bt.decorator.ForceSuccessNode;
import com.marswars.bt.decorator.InverterNode;
import com.marswars.bt.decorator.KeepRunningUntilFailureNode;
import com.marswars.bt.decorator.RepeatNode;
import com.marswars.bt.decorator.RetryNode;
import com.marswars.bt.decorator.RunOnceNode;
import com.marswars.bt.decorator.SubTreeNode;
import com.marswars.bt.decorator.TimeoutNode;
import com.marswars.bt.xml.BtXmlException;
import com.marswars.bt.xml.BtXmlParser;
import com.marswars.bt.xml.BtXmlWriter;
import com.marswars.logging.MwLog;
import edu.wpi.first.wpilibj2.command.Command;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Predicate;

/**
 * Registry of node types and builder of {@link BehaviorTree}s from BT.CPP v4 XML or {@link
 * TreeSpec}s, mirroring BT.CPP {@code BehaviorTreeFactory}.
 *
 * <p>All BT.CPP v4 built-ins without scripting are registered by default. Robot code registers its
 * own actions/conditions, e.g.
 *
 * <pre>{@code
 * BehaviorTreeFactory factory = new BehaviorTreeFactory();
 * factory.registerSetState("SetIntakeState", "Request an intake state", IntakeStates.class,
 *         s -> IntakeSubsystem.getInstance().setWantedState(s));
 * factory.registerSimpleCondition("IsShooterReady", "", List.of(),
 *         n -> ShooterSubsystem.getInstance().isShooterReady());
 * factory.registerNodeType("FollowTrajectory", NodeKind.ACTION, "...", ports,
 *         FollowTrajectoryNode::new);
 * BehaviorTree tree = factory.createTreeFromFile(path);
 * }</pre>
 *
 * <p>Instantiation validates the tree: unknown node IDs, unknown or missing ports, unparsable
 * literals, wrong child counts, Parallel thresholds and unknown/recursive SubTrees all throw a
 * {@link BtException} naming the node and source line.
 */
public final class BehaviorTreeFactory {

    /** Creates a node instance; children are attached by the factory afterwards. */
    @FunctionalInterface
    public interface NodeBuilder {
        TreeNode build(String name, NodeConfig config);
    }

    private record Registration(NodeModel model, NodeBuilder builder) {}

    /** BT.CPP v4 nodes MW-Lib deliberately does not implement (scripting / C++ queues / Groot). */
    public static final Set<String> UNSUPPORTED_BTCPP_NODES =
            Set.of(
                    "Script",
                    "ScriptCondition",
                    "Precondition",
                    "Switch2",
                    "Switch3",
                    "Switch4",
                    "Switch5",
                    "Switch6",
                    "LoopInt",
                    "LoopDouble",
                    "LoopString",
                    "LoopBool",
                    "ConsumeQueue",
                    "PopFromQueue",
                    "QueueSize",
                    "ManualSelector",
                    "TestNode",
                    "SkipUnlessUpdated",
                    "WaitValueUpdate",
                    "WasEntryUpdated",
                    "EntryUpdated");

    private final Map<String, Registration> registry_ = new LinkedHashMap<>();
    private final Map<String, NodeSpec> trees_ = new LinkedHashMap<>();
    private final Map<String, NodeModel> tree_models_ = new LinkedHashMap<>();
    private DoubleSupplier clock_;

    /** Factory on the replay-safe robot clock ({@link MwLog#timestampSeconds()}). */
    public BehaviorTreeFactory() {
        this(MwLog::timestampSeconds);
    }

    /** Factory on an explicit clock in seconds (tests, tools). */
    public BehaviorTreeFactory(DoubleSupplier clock) {
        clock_ = Objects.requireNonNull(clock, "clock");
        registerBuiltins();
    }

    public DoubleSupplier getClock() {
        return clock_;
    }

    public void setClock(DoubleSupplier clock) {
        clock_ = Objects.requireNonNull(clock, "clock");
    }

    // ------------------------------------------------------------------ registration

    /** Registers a node type. IDs are unique; registering an existing ID throws. */
    public void registerNodeType(NodeModel model, NodeBuilder builder) {
        Objects.requireNonNull(model, "model");
        Objects.requireNonNull(builder, "builder");
        if (registry_.containsKey(model.id())) {
            throw new BtException("Node ID '" + model.id() + "' is already registered");
        }
        if (NodeSpec.SUBTREE.equals(model.id()) && !registry_.isEmpty()) {
            throw new BtException("'SubTree' is reserved");
        }
        registry_.put(model.id(), new Registration(model, builder));
    }

    public void registerNodeType(
            String id,
            NodeKind kind,
            String description,
            List<PortInfo> ports,
            NodeBuilder builder) {
        registerNodeType(new NodeModel(id, kind, ports, description, NodeOrigin.ROBOT), builder);
    }

    /** Synchronous action from a function returning SUCCESS or FAILURE. */
    public void registerSimpleAction(
            String id,
            String description,
            List<PortInfo> ports,
            Function<TreeNode, NodeStatus> tick) {
        registerNodeType(
                id,
                NodeKind.ACTION,
                description,
                ports,
                (name, cfg) -> new SimpleActionNode(name, cfg, tick));
    }

    /** Synchronous action that always succeeds after running {@code action}. */
    public void registerInstantAction(
            String id, String description, List<PortInfo> ports, Consumer<TreeNode> action) {
        registerSimpleAction(
                id,
                description,
                ports,
                n -> {
                    action.accept(n);
                    return NodeStatus.SUCCESS;
                });
    }

    /** Condition from a predicate (true = SUCCESS). */
    public void registerSimpleCondition(
            String id, String description, List<PortInfo> ports, Predicate<TreeNode> predicate) {
        registerNodeType(
                id,
                NodeKind.CONDITION,
                description,
                ports,
                (name, cfg) -> new SimpleConditionNode(name, cfg, predicate));
    }

    /**
     * Instant action with one ENUM port {@code state}: parses it into {@code enumClass} and hands
     * it to {@code setter} (typically {@code subsystem.setWantedState}).
     */
    public <E extends Enum<E>> void registerSetState(
            String id, String description, Class<E> enumClass, Consumer<E> setter) {
        registerInstantAction(
                id,
                description,
                List.of(PortInfo.enumInput("state", enumClass, null, "State to request")),
                n -> setter.accept(n.getEnum("state", enumClass)));
    }

    /** Action that runs a fresh WPILib {@link Command} each time it starts. */
    public void registerCommand(
            String id,
            String description,
            List<PortInfo> ports,
            Function<TreeNode, Command> commandFactory) {
        registerNodeType(
                id,
                NodeKind.ACTION,
                description,
                ports,
                (name, cfg) -> new RunCommandNode(name, cfg, commandFactory));
    }

    public boolean isRegistered(String id) {
        return registry_.containsKey(id);
    }

    public Optional<NodeModel> getNodeModel(String id) {
        Registration r = registry_.get(id);
        return r == null ? Optional.empty() : Optional.of(r.model());
    }

    /** Every registered model in registration order (built-ins first). */
    public List<NodeModel> getNodeModels() {
        List<NodeModel> models = new ArrayList<>();
        for (Registration r : registry_.values()) {
            models.add(r.model());
        }
        return models;
    }

    /**
     * BT.CPP {@code writeTreeNodesModelXML}: the node-spec palette editors and Groot2 load. Without
     * built-ins it lists MW-Lib's shared nodes first and the robot's nodes appended after them,
     * each group under a comment.
     */
    public String writeTreeNodesModelXml(boolean includeBuiltins) {
        return includeBuiltins
                ? writeTreeNodesModelXml(java.util.EnumSet.allOf(NodeOrigin.class))
                : writeTreeNodesModelXml(java.util.EnumSet.complementOf(java.util.EnumSet.of(NodeOrigin.BTCPP)));
    }

    /** Node-spec document with the models of the given origins, grouped by origin. */
    public String writeTreeNodesModelXml(Set<NodeOrigin> origins) {
        Map<NodeOrigin, List<NodeModel>> groups = new java.util.EnumMap<>(NodeOrigin.class);
        for (NodeModel m : getNodeModels()) {
            if (origins.contains(m.origin()) && m.kind() != NodeKind.SUBTREE) {
                groups.computeIfAbsent(m.origin(), o -> new ArrayList<>()).add(m);
            }
        }
        Map<String, List<NodeModel>> sections = new LinkedHashMap<>();
        groups.forEach((origin, models) -> sections.put(origin.title(), models));
        return BtXmlWriter.writeModels(sections);
    }

    /** Registers an MW-Lib shared node (written to node-spec files under the MW-Lib section). */
    public void registerLibraryNode(
            String id, NodeKind kind, String description, List<PortInfo> ports, NodeBuilder builder) {
        registerNodeType(new NodeModel(id, kind, ports, description, NodeOrigin.MWLIB), builder);
    }

    /** Non-builtin models referenced by {@code spec} (what to embed when saving that file). */
    public List<NodeModel> modelsUsedBy(TreeSpec spec) {
        Set<String> ids = new LinkedHashSet<>();
        for (NodeSpec root : spec.trees().values()) {
            collectIds(root, ids);
        }
        List<NodeModel> models = new ArrayList<>();
        for (String id : ids) {
            getNodeModel(id).filter(m -> !m.builtin()).ifPresent(models::add);
        }
        return models;
    }

    private static void collectIds(NodeSpec node, Set<String> out) {
        out.add(node.id());
        for (NodeSpec c : node.children()) {
            collectIds(c, out);
        }
    }

    /** JSON array of every model, with ports, defaults and enum choices (for tools/editors). */
    public String nodeModelsJson() {
        JsonArray array = new JsonArray();
        for (NodeModel m : getNodeModels()) {
            JsonObject o = new JsonObject();
            o.addProperty("id", m.id());
            o.addProperty("category", m.kind().xmlTag());
            o.addProperty("builtin", m.builtin());
            o.addProperty("origin", m.origin().name().toLowerCase());
            o.addProperty("description", m.description());
            JsonArray ports = new JsonArray();
            for (PortInfo p : m.ports()) {
                JsonObject po = new JsonObject();
                po.addProperty("name", p.name());
                po.addProperty("direction", p.direction().name().toLowerCase());
                po.addProperty("type", p.xmlType());
                if (p.defaultValue() != null) {
                    po.addProperty("default", p.defaultValue());
                }
                po.addProperty("required", p.required());
                po.addProperty("description", p.description());
                if (!p.choices().isEmpty()) {
                    JsonArray choices = new JsonArray();
                    p.choices().forEach(choices::add);
                    po.add("choices", choices);
                }
                ports.add(po);
            }
            o.add("ports", ports);
            array.add(o);
        }
        Gson gson = new GsonBuilder().disableHtmlEscaping().create();
        return gson.toJson(array);
    }

    // ------------------------------------------------------------------ tree definitions

    /** Parses XML and remembers its trees (for later {@link #createTree(String)} / SubTrees). */
    public TreeSpec registerBehaviorTreeFromText(String xml) {
        TreeSpec spec = BtXmlParser.parse(xml);
        registerTrees(spec);
        return spec;
    }

    public TreeSpec registerBehaviorTreeFromFile(Path file) {
        TreeSpec spec = BtXmlParser.parse(file);
        registerTrees(spec);
        return spec;
    }

    /** Remembers every tree of {@code spec}; later definitions replace earlier ones. */
    public void registerTrees(TreeSpec spec) {
        trees_.putAll(spec.trees());
        for (NodeModel m : spec.models()) {
            if (m.kind() == NodeKind.SUBTREE) {
                tree_models_.put(m.id(), m);
            }
        }
    }

    public Set<String> registeredTreeIds() {
        return trees_.keySet();
    }

    // ------------------------------------------------------------------ instantiation

    public BehaviorTree createTreeFromText(String xml) {
        return createTree(BtXmlParser.parse(xml), Blackboard.createRoot(), xml);
    }

    public BehaviorTree createTreeFromFile(Path file) {
        TreeSpec spec = BtXmlParser.parse(file);
        return createTree(spec, Blackboard.createRoot(), BtXmlWriter.write(spec, List.of()));
    }

    /** Instantiates a previously registered tree. */
    public BehaviorTree createTree(String treeId) {
        NodeSpec root = trees_.get(treeId);
        if (root == null) {
            throw new BtException("No registered tree with ID '" + treeId + "'");
        }
        TreeSpec spec = new TreeSpec(treeId, trees_, List.of(), List.of());
        return createTree(spec, Blackboard.createRoot(), null);
    }

    public BehaviorTree createTree(TreeSpec spec) {
        return createTree(spec, Blackboard.createRoot(), null);
    }

    /**
     * Instantiates {@code spec}'s main tree on {@code rootBlackboard} (pre-populate it, e.g. with
     * {@code @trajectories}). {@code xml} is the text to report to monitors; when null the spec is
     * re-serialized.
     */
    public BehaviorTree createTree(TreeSpec spec, Blackboard rootBlackboard, String xml) {
        if (spec.mainTreeId() == null || spec.mainTreeId().isEmpty()) {
            throw new BtException(
                    "Several <BehaviorTree> are defined but main_tree_to_execute is not set");
        }
        NodeSpec root = spec.mainTree();
        BuildContext ctx = new BuildContext(spec);
        // Tree parameters: main-tree ports declared in <TreeNodesModel> seed the root blackboard
        // with their defaults unless the caller already provided a value.
        ctx.treeModel(spec.mainTreeId())
                .ifPresent(m -> applyPortDefaults(m, rootBlackboard, java.util.Set.of()));
        Deque<String> stack = new ArrayDeque<>();
        stack.push(spec.mainTreeId());
        TreeNode root_node = build(root, ctx, rootBlackboard, "", stack);
        return new BehaviorTree(
                spec.mainTreeId(),
                root_node,
                rootBlackboard,
                clock_,
                xml != null ? xml : BtXmlWriter.write(spec, List.of()),
                ctx.subtrees);
    }

    private final class BuildContext {
        final TreeSpec spec;
        final List<BehaviorTree.Subtree> subtrees = new ArrayList<>();
        int uid_counter = 0;

        BuildContext(TreeSpec spec) {
            this.spec = spec;
        }

        NodeSpec lookupTree(String id) {
            NodeSpec n = spec.trees().get(id);
            return n != null ? n : trees_.get(id);
        }

        Optional<NodeModel> treeModel(String id) {
            Optional<NodeModel> m = spec.treeModel(id);
            return m.isPresent() ? m : Optional.ofNullable(tree_models_.get(id));
        }
    }

    private BtXmlException error(NodeSpec spec, String path, String message) {
        String where = path == null || path.isEmpty() ? spec.typeId() : path;
        return new BtXmlException("[" + where + "] " + message, spec.line());
    }

    private TreeNode build(
            NodeSpec spec, BuildContext ctx, Blackboard bb, String prefix, Deque<String> stack) {
        if (spec.isSubTree()) {
            return buildSubTree(spec, ctx, bb, prefix, stack);
        }
        Registration reg = registry_.get(spec.id());
        if (reg == null) {
            if (UNSUPPORTED_BTCPP_NODES.contains(spec.id())) {
                throw error(
                        spec,
                        null,
                        "BT.CPP node '" + spec.id() + "' is not supported by MW-Lib (no scripting)");
            }
            throw error(spec, null, "unknown node ID '" + spec.id() + "' (not registered)");
        }
        NodeModel model = reg.model();

        final int uid = ++ctx.uid_counter;
        final String instance_name = spec.name() != null ? spec.name() : spec.id();
        final String path =
                prefix + instance_name + (instance_name.equals(spec.id()) ? "::" + uid : "");

        checkArity(spec, model, path);
        Map<String, String> inputs = new HashMap<>();
        Map<String, String> outputs = new HashMap<>();
        resolvePorts(spec, model, path, inputs, outputs);

        NodeConfig cfg = new NodeConfig(bb, inputs, outputs, clock_, path, model);
        TreeNode node;
        try {
            node = reg.builder().build(instance_name, cfg);
        } catch (BtException e) {
            throw error(spec, path, e.getMessage());
        }
        if (node.kind() != model.kind()) {
            throw error(
                    spec,
                    path,
                    "builder for '" + spec.id() + "' made a " + node.kind() + ", model says "
                            + model.kind());
        }

        if (node instanceof ControlNode control) {
            for (NodeSpec child : spec.children()) {
                control.addChild(build(child, ctx, bb, prefix, stack));
            }
        } else if (node instanceof DecoratorNode decorator) {
            decorator.setChild(build(spec.children().get(0), ctx, bb, prefix, stack));
        }
        return node;
    }

    private TreeNode buildSubTree(
            NodeSpec spec, BuildContext ctx, Blackboard bb, String prefix, Deque<String> stack) {
        String sid = spec.subtreeId();
        NodeSpec target = ctx.lookupTree(sid);
        if (target == null) {
            throw error(spec, null, "SubTree ID '" + sid + "' is not a defined <BehaviorTree>");
        }
        if (stack.contains(sid)) {
            throw error(spec, null, "recursive SubTree '" + sid + "'");
        }
        if (!spec.children().isEmpty()) {
            throw error(spec, null, "a <SubTree> reference cannot have children");
        }
        final int uid = ++ctx.uid_counter;
        final String instance_name = spec.name() != null ? spec.name() : sid;
        final String path = prefix + instance_name + (instance_name.equals(sid) ? "::" + uid : "");

        Blackboard child_bb = Blackboard.createChild(bb);
        for (Map.Entry<String, String> attr : spec.attributes().entrySet()) {
            String key = attr.getKey();
            String value = attr.getValue();
            if ("_autoremap".equals(key)) {
                child_bb.enableAutoRemapping(
                        PortValues.parseLiteral(value, Boolean.class, "_autoremap of " + path));
            } else if (PortValues.isPointer(value)) {
                child_bb.addSubtreeRemapping(key, PortValues.pointerKey(value, key));
            } else {
                child_bb.setLocal(key, value);
            }
        }

        // BT.CPP 4.6: ports of the subtree's model that the <SubTree> element does not set take
        // their default value.
        ctx.treeModel(sid)
                .ifPresent(m -> applyPortDefaults(m, child_bb, spec.attributes().keySet()));

        NodeModel model = registry_.get(NodeSpec.SUBTREE).model();
        NodeConfig cfg =
                new NodeConfig(bb, spec.attributes(), Map.of(), clock_, path, model);
        SubTreeNode node = new SubTreeNode(instance_name, cfg, sid);
        ctx.subtrees.add(new BehaviorTree.Subtree(path, sid, child_bb));

        stack.push(sid);
        node.setChild(build(target, ctx, child_bb, path + "/", stack));
        stack.pop();
        return node;
    }

    /** Sets each defaulted port of {@code model} not in {@code given} and not already present. */
    private static void applyPortDefaults(NodeModel model, Blackboard bb, java.util.Set<String> given) {
        for (PortInfo port : model.ports()) {
            if (port.defaultValue() != null
                    && !given.contains(port.name())
                    && !bb.contains(port.name())) {
                bb.setLocal(port.name(), ParameterStore.typed(port, port.defaultValue()));
            }
        }
    }

    private void checkArity(NodeSpec spec, NodeModel model, String path) {
        int n = spec.children().size();
        switch (model.kind()) {
            case ACTION, CONDITION -> {
                if (n != 0) {
                    throw error(spec, path, model.kind().xmlTag() + " nodes cannot have children");
                }
            }
            case DECORATOR -> {
                if (n != 1) {
                    throw error(spec, path, "a Decorator needs exactly one child, found " + n);
                }
            }
            case CONTROL -> {
                if (n == 0) {
                    throw error(spec, path, "a Control node needs at least one child");
                }
                if (("IfThenElse".equals(spec.id()) || "WhileDoElse".equals(spec.id()))
                        && (n < 2 || n > 3)) {
                    throw error(spec, path, spec.id() + " needs 2 or 3 children, found " + n);
                }
            }
            default -> {}
        }
    }

    private void resolvePorts(
            NodeSpec spec,
            NodeModel model,
            String path,
            Map<String, String> inputs,
            Map<String, String> outputs) {
        for (Map.Entry<String, String> attr : spec.attributes().entrySet()) {
            PortInfo port =
                    model.port(attr.getKey())
                            .orElseThrow(
                                    () ->
                                            error(
                                                    spec,
                                                    path,
                                                    "unknown port '"
                                                            + attr.getKey()
                                                            + "' for "
                                                            + spec.id()
                                                            + " (ports: "
                                                            + portNames(model)
                                                            + ")"));
            String value = attr.getValue();
            switch (port.direction()) {
                case OUTPUT -> outputs.put(port.name(), value);
                case INOUT -> {
                    inputs.put(port.name(), value);
                    outputs.put(port.name(), value);
                }
                case INPUT -> {
                    inputs.put(port.name(), value);
                    if (!PortValues.isPointer(value)) {
                        checkLiteral(spec, path, port, value);
                    }
                }
            }
        }
        for (PortInfo port : model.ports()) {
            if (port.required() && !spec.attributes().containsKey(port.name())) {
                throw error(spec, path, "missing required port '" + port.name() + "'");
            }
        }
        if ("Parallel".equals(spec.id())) {
            int children = spec.children().size();
            int success = literalInt(inputs, ParallelNode.SUCCESS_COUNT, -1);
            int failure = literalInt(inputs, ParallelNode.FAILURE_COUNT, 1);
            if (threshold(success, children) > children) {
                throw error(spec, path, "success_count " + success + " > " + children + " children");
            }
            if (threshold(failure, children) > children) {
                throw error(spec, path, "failure_count " + failure + " > " + children + " children");
            }
        }
    }

    private static int threshold(int value, int children) {
        return value < 0 ? Math.max(children + value + 1, 0) : value;
    }

    private static int literalInt(Map<String, String> inputs, String key, int fallback) {
        String v = inputs.get(key);
        if (v == null || PortValues.isPointer(v)) {
            return fallback;
        }
        return Integer.parseInt(v.trim());
    }

    private void checkLiteral(NodeSpec spec, String path, PortInfo port, String value) {
        String context = "port '" + port.name() + "'";
        try {
            switch (port.type()) {
                case DOUBLE -> PortValues.parseLiteral(value, Double.class, context);
                case INT -> PortValues.parseLiteral(value, Integer.class, context);
                case BOOLEAN -> PortValues.parseLiteral(value, Boolean.class, context);
                case ENUM -> {
                    if (!port.choices().isEmpty() && !port.choices().contains(value.trim())) {
                        throw new BtException(
                                "invalid "
                                        + context
                                        + " value '"
                                        + value
                                        + "'; expected one of "
                                        + port.choices());
                    }
                }
                default -> {}
            }
        } catch (BtException e) {
            throw error(spec, path, e.getMessage());
        }
    }

    private static String portNames(NodeModel model) {
        List<String> names = new ArrayList<>();
        for (PortInfo p : model.ports()) {
            names.add(p.name());
        }
        return names.isEmpty() ? "none" : String.join(", ", names);
    }

    // ------------------------------------------------------------------ built-ins

    private void builtin(String id, NodeKind kind, String desc, List<PortInfo> ports, NodeBuilder b) {
        registerNodeType(new NodeModel(id, kind, ports, desc, NodeOrigin.BTCPP), b);
    }

    private void registerBuiltins() {
        final NodeKind CONTROL = NodeKind.CONTROL;
        final NodeKind DECORATOR = NodeKind.DECORATOR;
        final NodeKind ACTION = NodeKind.ACTION;

        builtin(NodeSpec.SUBTREE, NodeKind.SUBTREE, "Instance of another BehaviorTree", List.of(),
                (n, c) -> {
                    throw new BtException("SubTree nodes are created from <SubTree ID=...>");
                });

        builtin("Sequence", CONTROL, "Tick children in order; fail on the first failure",
                List.of(), SequenceNode::new);
        builtin("ReactiveSequence", CONTROL,
                "Sequence that re-checks earlier children every tick", List.of(),
                ReactiveSequenceNode::new);
        builtin("SequenceWithMemory", CONTROL,
                "Sequence that resumes at the failed child instead of restarting", List.of(),
                SequenceWithMemoryNode::new);
        builtin("Fallback", CONTROL, "Try children in order until one succeeds", List.of(),
                FallbackNode::new);
        builtin("ReactiveFallback", CONTROL,
                "Fallback that re-checks earlier children every tick", List.of(),
                ReactiveFallbackNode::new);
        builtin("Parallel", CONTROL, "Tick all children; succeed/fail on thresholds",
                List.of(
                        PortInfo.input(ParallelNode.SUCCESS_COUNT, PortType.INT, "-1",
                                "Successes needed to succeed (-1 = all)"),
                        PortInfo.input(ParallelNode.FAILURE_COUNT, PortType.INT, "1",
                                "Failures needed to fail")),
                ParallelNode::new);
        builtin("ParallelAll", CONTROL, "Run all children to completion",
                List.of(PortInfo.input(ParallelAllNode.MAX_FAILURES, PortType.INT, "1",
                        "Failures that make the node fail")),
                ParallelAllNode::new);
        registerLibraryNode("ParallelDeadline", CONTROL,
                "Run all children until the first one finishes, then halt the rest and return its"
                        + " result",
                List.of(), ParallelDeadlineNode::new);
        builtin("IfThenElse", CONTROL, "if child0 then child1 else child2", List.of(),
                IfThenElseNode::new);
        builtin("WhileDoElse", CONTROL,
                "Reactive if: re-checks child0 every tick, halting the other branch", List.of(),
                WhileDoElseNode::new);

        builtin("Inverter", DECORATOR, "Swap SUCCESS and FAILURE", List.of(), InverterNode::new);
        builtin("ForceSuccess", DECORATOR, "SUCCESS once the child completes", List.of(),
                ForceSuccessNode::new);
        builtin("ForceFailure", DECORATOR, "FAILURE once the child completes", List.of(),
                ForceFailureNode::new);
        builtin("Repeat", DECORATOR, "Repeat the child num_cycles times (-1 = forever)",
                List.of(PortInfo.input(RepeatNode.NUM_CYCLES, PortType.INT,
                        "Repeat a successful child up to N times. Use -1 for infinite loops")),
                RepeatNode::new);
        builtin("RetryUntilSuccessful", DECORATOR, "Retry a failing child (-1 = forever)",
                List.of(PortInfo.input(RetryNode.NUM_ATTEMPTS, PortType.INT,
                        "Execute again a failing child up to N times. Use -1 for infinite loops")),
                RetryNode::new);
        builtin("KeepRunningUntilFailure", DECORATOR,
                "Restart the child on success; RUNNING until it fails", List.of(),
                KeepRunningUntilFailureNode::new);
        builtin("Timeout", DECORATOR, "Halt the child and fail after msec",
                List.of(PortInfo.input(TimeoutNode.MSEC, PortType.INT,
                        "After a certain amount of time, halt() the child if it is still running.")),
                TimeoutNode::new);
        builtin("Delay", DECORATOR, "Wait delay_msec before ticking the child",
                List.of(PortInfo.input(DelayNode.DELAY_MSEC, PortType.INT,
                        "Tick the child after a few milliseconds")),
                DelayNode::new);
        builtin("RunOnce", DECORATOR, "Run the child only once",
                List.of(PortInfo.input(RunOnceNode.THEN_SKIP, PortType.BOOLEAN, "true",
                        "If true, skip after the first execution, otherwise return the same "
                                + "NodeStatus returned once by the child.")),
                RunOnceNode::new);

        builtin("AlwaysSuccess", ACTION, "Return SUCCESS", List.of(), AlwaysSuccessNode::new);
        builtin("AlwaysFailure", ACTION, "Return FAILURE", List.of(), AlwaysFailureNode::new);
        builtin("Sleep", ACTION, "Wait msec milliseconds",
                List.of(PortInfo.input(SleepNode.MSEC, PortType.INT, "Milliseconds to wait")),
                SleepNode::new);
        builtin("SetBlackboard", ACTION, "Write a value into the blackboard",
                List.of(
                        PortInfo.input(SetBlackboardNode.VALUE, PortType.STRING,
                                "Value to write (literal or {key})"),
                        PortInfo.inout(SetBlackboardNode.OUTPUT_KEY, PortType.STRING,
                                "Name of the blackboard entry where the value should be written")),
                SetBlackboardNode::new);
        builtin("UnsetBlackboard", ACTION, "Remove a blackboard entry",
                List.of(PortInfo.input(UnsetBlackboardNode.KEY, PortType.STRING,
                        "Key of the entry to remove")),
                UnsetBlackboardNode::new);
    }
}
