package com.marswars.bt.core;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A parsed BT.CPP v4 document: every {@code <BehaviorTree>} (ID to root node), which one is the
 * main tree, the {@code <TreeNodesModel>} entries found in the file, and parser warnings.
 *
 * @param mainTreeId tree to execute ({@code main_tree_to_execute}, or the only tree)
 * @param trees tree ID to root node, in document order
 * @param models node models declared in the file (informational)
 * @param warnings non-fatal issues found while parsing
 */
public record TreeSpec(
        String mainTreeId, Map<String, NodeSpec> trees, List<NodeModel> models, List<String> warnings) {

    public TreeSpec {
        trees = Collections.unmodifiableMap(new LinkedHashMap<>(trees));
        models = models == null ? List.of() : List.copyOf(models);
        warnings = warnings == null ? List.of() : List.copyOf(warnings);
    }

    /**
     * The {@code <SubTree ID=treeId>} model declared in this file's {@code <TreeNodesModel>}: the
     * ports of that tree (its parameters), if any.
     */
    public java.util.Optional<NodeModel> treeModel(String treeId) {
        for (NodeModel m : models) {
            if (m.kind() == NodeKind.SUBTREE && m.id().equals(treeId)) {
                return java.util.Optional.of(m);
            }
        }
        return java.util.Optional.empty();
    }

    /** Ports of the main tree (see {@link #treeModel}); empty when none are declared. */
    public List<PortInfo> mainTreeParameters() {
        return treeModel(mainTreeId).map(NodeModel::ports).orElse(List.of());
    }

    public NodeSpec mainTree() {
        NodeSpec root = trees.get(mainTreeId);
        if (root == null) {
            throw new BtException("Main tree '" + mainTreeId + "' is not defined");
        }
        return root;
    }
}
