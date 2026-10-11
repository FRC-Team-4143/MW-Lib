package com.marswars.auto;

import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.NodeModel;
import com.marswars.bt.core.NodeSpec;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.PortType;
import com.marswars.bt.core.PortValues;
import com.marswars.bt.core.TreeSpec;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/** Finds the Choreo trajectories a tree uses, so its auto can load them before it runs. */
public final class TrajectoryNames {
    private TrajectoryNames() {}

    /**
     * Literal values of TRAJECTORY ports in the main tree, in document order and without
     * duplicates. SubTrees are expanded in place; a {@code {key}} inside a SubTree resolves to a
     * literal its {@code <SubTree>} element passes for that key.
     */
    public static List<String> discover(TreeSpec spec, BehaviorTreeFactory factory) {
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
}
