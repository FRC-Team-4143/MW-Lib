package com.marswars.bt.xml;

import com.marswars.bt.core.NodeModel;
import com.marswars.bt.core.NodeSpec;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.TreeSpec;
import java.util.Collection;
import java.util.Map;

/**
 * Writes {@link TreeSpec}s and node models as BehaviorTree.CPP v4 XML (2-space indent, attributes
 * in source order, trailing newline) so files round-trip and open in Groot2.
 */
public final class BtXmlWriter {
    private static final String INDENT = "  ";

    private BtXmlWriter() {}

    /** Full document: every tree, plus a {@code <TreeNodesModel>} for {@code models} (may be empty). */
    public static String write(TreeSpec spec, Collection<NodeModel> models) {
        StringBuilder sb = new StringBuilder();
        sb.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        sb.append("<root BTCPP_format=\"4\"");
        if (spec.mainTreeId() != null && !spec.mainTreeId().isEmpty()) {
            attr(sb, "main_tree_to_execute", spec.mainTreeId());
        }
        sb.append(">\n");
        boolean first = true;
        for (Map.Entry<String, NodeSpec> tree : spec.trees().entrySet()) {
            if (!first) {
                sb.append('\n');
            }
            first = false;
            sb.append(INDENT).append("<BehaviorTree");
            attr(sb, "ID", tree.getKey());
            sb.append(">\n");
            writeNode(sb, tree.getValue(), 2);
            sb.append(INDENT).append("</BehaviorTree>\n");
        }
        if (models != null && !models.isEmpty()) {
            sb.append('\n');
            writeModels(sb, models, 1);
        }
        sb.append("</root>\n");
        return sb.toString();
    }

    /**
     * Standalone node-spec document ({@code <root BTCPP_format="4"><TreeNodesModel>...}), the format of
     * BT.CPP {@code writeTreeNodesModelXML} that editors load as a palette.
     */
    public static String writeModels(Collection<NodeModel> models) {
        StringBuilder sb = new StringBuilder();
        sb.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        sb.append("<root BTCPP_format=\"4\">\n");
        writeModels(sb, models, 1);
        sb.append("</root>\n");
        return sb.toString();
    }

    private static void writeNode(StringBuilder sb, NodeSpec node, int depth) {
        indent(sb, depth);
        sb.append('<').append(node.id());
        if (node.isSubTree()) {
            attr(sb, "ID", node.subtreeId());
        }
        if (node.name() != null) {
            attr(sb, "name", node.name());
        }
        for (Map.Entry<String, String> port : node.attributes().entrySet()) {
            attr(sb, port.getKey(), port.getValue());
        }
        if (node.children().isEmpty()) {
            sb.append("/>\n");
            return;
        }
        sb.append(">\n");
        for (NodeSpec child : node.children()) {
            writeNode(sb, child, depth + 1);
        }
        indent(sb, depth);
        sb.append("</").append(node.id()).append(">\n");
    }

    private static void writeModels(StringBuilder sb, Collection<NodeModel> models, int depth) {
        indent(sb, depth);
        sb.append("<TreeNodesModel>\n");
        for (NodeModel model : models) {
            indent(sb, depth + 1);
            sb.append('<').append(model.kind().xmlTag());
            attr(sb, "ID", model.id());
            if (model.ports().isEmpty() && model.description().isEmpty()) {
                sb.append("/>\n");
                continue;
            }
            sb.append(">\n");
            for (PortInfo port : model.ports()) {
                indent(sb, depth + 2);
                sb.append('<').append(port.direction().xmlTag());
                attr(sb, "name", port.name());
                if (!port.xmlType().isEmpty()) {
                    attr(sb, "type", port.xmlType());
                }
                if (port.defaultValue() != null) {
                    attr(sb, "default", port.defaultValue());
                }
                if (port.description().isEmpty()) {
                    sb.append("/>\n");
                } else {
                    sb.append('>')
                            .append(escape(port.description()))
                            .append("</")
                            .append(port.direction().xmlTag())
                            .append(">\n");
                }
            }
            if (!model.description().isEmpty()) {
                indent(sb, depth + 2);
                sb.append("<MetadataFields>\n");
                indent(sb, depth + 3);
                sb.append("<Metadata");
                attr(sb, "description", model.description());
                sb.append("/>\n");
                indent(sb, depth + 2);
                sb.append("</MetadataFields>\n");
            }
            indent(sb, depth + 1);
            sb.append("</").append(model.kind().xmlTag()).append(">\n");
        }
        indent(sb, depth);
        sb.append("</TreeNodesModel>\n");
    }

    private static void indent(StringBuilder sb, int depth) {
        sb.append(INDENT.repeat(depth));
    }

    private static void attr(StringBuilder sb, String key, String value) {
        sb.append(' ').append(key).append("=\"").append(escape(value)).append('"');
    }

    /** XML-escapes text and attribute values. */
    public static String escape(String s) {
        StringBuilder out = new StringBuilder(s.length());
        for (char c : s.toCharArray()) {
            switch (c) {
                case '&' -> out.append("&amp;");
                case '<' -> out.append("&lt;");
                case '>' -> out.append("&gt;");
                case '"' -> out.append("&quot;");
                case '\'' -> out.append("&apos;");
                default -> out.append(c);
            }
        }
        return out.toString();
    }
}
