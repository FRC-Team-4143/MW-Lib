package com.marswars.bt.core;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Parsed (not yet instantiated) node: what the XML or {@link TreeBuilder} says, before the factory
 * resolves it against registered node types.
 *
 * @param id registration ID ({@code "SubTree"} for subtree references)
 * @param name explicit instance name ({@code name="..."}), or {@code null}
 * @param attributes port attributes in document order (for a SubTree: its remappings, plus {@code
 *     _autoremap})
 * @param children child specs in tick order
 * @param subtreeId for SubTree references, the ID of the referenced {@code <BehaviorTree>}
 * @param line 1-based source line, or 0 when unknown
 */
public record NodeSpec(
        String id,
        String name,
        Map<String, String> attributes,
        List<NodeSpec> children,
        String subtreeId,
        int line) {

    public static final String SUBTREE = "SubTree";

    public NodeSpec {
        Objects.requireNonNull(id, "id");
        attributes =
                attributes == null
                        ? Map.of()
                        : java.util.Collections.unmodifiableMap(new LinkedHashMap<>(attributes));
        children = children == null ? List.of() : List.copyOf(children);
    }

    public boolean isSubTree() {
        return SUBTREE.equals(id);
    }

    /** The type shown to users: the referenced tree ID for subtrees, else the registration ID. */
    public String typeId() {
        return isSubTree() ? subtreeId : id;
    }
}
