package com.marswars.bt.xml;

import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.NodeKind;
import com.marswars.bt.core.NodeModel;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.TreeNode;
import com.marswars.bt.decorator.SubTreeNode;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

/**
 * Writes an instantiated tree the way BehaviorTree.CPP's {@code WriteTreeToXML(tree,
 * add_metadata=true, add_builtin_models=true)} does. This is the XML embedded in {@code .btlog}
 * files, which Groot2 and the BT editor use to map transitions to nodes:
 *
 * <ul>
 *   <li>every node is written by registration ID with {@code name}, {@code _uid} and its ports
 *       (defaults included)
 *   <li>every SubTree <i>instance</i> gets its own {@code <BehaviorTree ID _fullpath>}, main tree
 *       first with {@code _fullpath=""}
 *   <li>{@code <TreeNodesModel>} lists the model of every node type used, built-ins included
 * </ul>
 */
public final class BtcppTreeXml {
    private static final String INDENT = "    ";

    private BtcppTreeXml() {}

    public static String write(BehaviorTree tree) {
        StringBuilder sb = new StringBuilder();
        sb.append("<root BTCPP_format=\"4\">\n");

        writeTree(sb, tree.getMainTreeId(), "", tree.getRoot());
        for (TreeNode node : tree.getNodes()) {
            if (node instanceof SubTreeNode st && st.hasChild()) {
                writeTree(sb, st.subtreeId(), node.getPath(), st.child());
            }
        }

        Map<String, NodeModel> models = new LinkedHashMap<>();
        for (TreeNode node : tree.getNodes()) {
            node.getModel()
                    .filter(m -> m.kind() != NodeKind.SUBTREE)
                    .ifPresent(m -> models.putIfAbsent(m.id(), m));
        }
        sb.append(INDENT).append("<TreeNodesModel>\n");
        for (NodeModel m : models.values()) {
            writeModel(sb, m);
        }
        sb.append(INDENT).append("</TreeNodesModel>\n");
        sb.append("</root>\n");
        return sb.toString();
    }

    private static void writeTree(StringBuilder sb, String id, String fullPath, TreeNode root) {
        sb.append(INDENT).append("<BehaviorTree");
        attr(sb, "ID", id);
        attr(sb, "_fullpath", fullPath);
        sb.append(">\n");
        writeNode(sb, root, 2);
        sb.append(INDENT).append("</BehaviorTree>\n");
    }

    private static void writeNode(StringBuilder sb, TreeNode node, int depth) {
        sb.append(INDENT.repeat(depth));
        if (node instanceof SubTreeNode st) {
            sb.append("<SubTree");
            attr(sb, "ID", st.subtreeId());
            attr(sb, "_fullpath", node.getPath());
            attr(sb, "_uid", Integer.toString(node.getUid()));
            for (Map.Entry<String, String> port : node.getConfig().inputPorts().entrySet()) {
                attr(sb, port.getKey(), port.getValue());
            }
            sb.append("/>\n"); // the instance's content is written as its own <BehaviorTree>
            return;
        }
        sb.append('<').append(node.getRegistrationId());
        attr(sb, "name", node.getName());
        attr(sb, "_uid", Integer.toString(node.getUid()));
        for (Map.Entry<String, String> port : ports(node).entrySet()) {
            attr(sb, port.getKey(), port.getValue());
        }
        if (node.childNodes().isEmpty()) {
            sb.append("/>\n");
            return;
        }
        sb.append(">\n");
        for (TreeNode child : node.childNodes()) {
            writeNode(sb, child, depth + 1);
        }
        sb.append(INDENT.repeat(depth)).append("</").append(node.getRegistrationId()).append(">\n");
    }

    /** Configured ports plus model defaults for unset ones, in model order. */
    private static Map<String, String> ports(TreeNode node) {
        Map<String, String> out = new LinkedHashMap<>();
        Map<String, String> in = node.getConfig().inputPorts();
        Map<String, String> outputs = node.getConfig().outputPorts();
        Set<String> seen = new LinkedHashSet<>();
        node.getModel()
                .ifPresent(
                        m -> {
                            for (PortInfo p : m.ports()) {
                                String v = in.containsKey(p.name()) ? in.get(p.name()) : outputs.get(p.name());
                                if (v == null) {
                                    v = p.defaultValue();
                                }
                                if (v != null) {
                                    out.put(p.name(), v);
                                }
                                seen.add(p.name());
                            }
                        });
        in.forEach((k, v) -> { if (!seen.contains(k)) out.put(k, v); });
        outputs.forEach((k, v) -> { if (!seen.contains(k)) out.putIfAbsent(k, v); });
        return out;
    }

    private static void writeModel(StringBuilder sb, NodeModel m) {
        sb.append(INDENT.repeat(2)).append('<').append(m.kind().xmlTag());
        attr(sb, "ID", m.id());
        if (m.ports().isEmpty()) {
            sb.append("/>\n");
            return;
        }
        sb.append(">\n");
        for (PortInfo p : m.ports()) {
            sb.append(INDENT.repeat(3)).append('<').append(p.direction().xmlTag());
            attr(sb, "name", p.name());
            if (!p.xmlType().isEmpty()) {
                attr(sb, "type", p.xmlType());
            }
            if (p.defaultValue() != null) {
                attr(sb, "default", p.defaultValue());
            }
            if (p.description().isEmpty()) {
                sb.append("/>\n");
            } else {
                sb.append('>')
                        .append(BtXmlWriter.escape(p.description()))
                        .append("</")
                        .append(p.direction().xmlTag())
                        .append(">\n");
            }
        }
        sb.append(INDENT.repeat(2)).append("</").append(m.kind().xmlTag()).append(">\n");
    }

    private static void attr(StringBuilder sb, String key, String value) {
        sb.append(' ').append(key).append("=\"").append(BtXmlWriter.escape(value)).append('"');
    }
}
