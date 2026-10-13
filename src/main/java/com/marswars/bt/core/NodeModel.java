package com.marswars.bt.core;

import java.util.List;
import java.util.Objects;
import java.util.Optional;

/**
 * Description of a registered node type: what the XML calls it, what kind it is, and its ports.
 * This is what gets written to {@code <TreeNodesModel>} and published to editors.
 *
 * @param id registration ID (the XML tag or {@code ID} attribute)
 * @param kind node category
 * @param ports declared ports in declaration order
 * @param description human-readable description
 * @param origin where the node type comes from (BT.CPP, MW-Lib, robot, or an XML file)
 */
public record NodeModel(
        String id, NodeKind kind, List<PortInfo> ports, String description, NodeOrigin origin) {

    public NodeModel {
        Objects.requireNonNull(id, "node id");
        Objects.requireNonNull(kind, "node kind");
        ports = ports == null ? List.of() : List.copyOf(ports);
        description = description == null ? "" : description;
        origin = origin == null ? NodeOrigin.ROBOT : origin;
    }

    /** True for BehaviorTree.CPP built-ins, which node-spec files leave out. */
    public boolean builtin() {
        return origin == NodeOrigin.BTCPP;
    }

    public Optional<PortInfo> port(String name) {
        for (PortInfo p : ports) {
            if (p.name().equals(name)) {
                return Optional.of(p);
            }
        }
        return Optional.empty();
    }
}
