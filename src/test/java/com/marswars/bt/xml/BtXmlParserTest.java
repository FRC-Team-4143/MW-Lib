package com.marswars.bt.xml;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.bt.core.NodeKind;
import com.marswars.bt.core.NodeSpec;
import com.marswars.bt.core.PortType;
import com.marswars.bt.core.TreeSpec;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;

class BtXmlParserTest {
    /** Fixture from the test classpath (independent of the test working directory). */
    static final Path SAMPLE = resource("/com/marswars/bt/sample_tree.xml");

    static Path resource(String name) {
        try {
            return Path.of(BtXmlParserTest.class.getResource(name).toURI());
        } catch (java.net.URISyntaxException e) {
            throw new IllegalStateException(e);
        }
    }

    @Test
    void parsesSampleWithIncludeAndBothNodeForms() {
        TreeSpec spec = BtXmlParser.parse(SAMPLE);
        assertEquals("MainTree", spec.mainTreeId());
        assertEquals(List.of("Shared", "MainTree", "Inner"), List.copyOf(spec.trees().keySet()));

        NodeSpec root = spec.mainTree();
        assertEquals("Sequence", root.id());
        assertEquals("root", root.name());
        assertTrue(root.attributes().isEmpty(), "Groot2 metadata attributes dropped");
        assertEquals(5, root.children().size());

        NodeSpec typed = root.children().get(1);
        assertEquals("RecordInput", typed.id(), "<Action ID=...> form");
        assertEquals("{speed}", typed.attributes().get("value"));

        NodeSpec sub = root.children().get(2);
        assertTrue(sub.isSubTree());
        assertEquals("Inner", sub.subtreeId());
        assertEquals("{speed}", sub.attributes().get("target"));
        assertEquals("false", sub.attributes().get("_autoremap"));
        assertNull(sub.name());

        assertEquals("SequenceWithMemory", root.children().get(4).id(), "deprecated alias mapped");
        assertTrue(spec.warnings().stream().anyMatch(w -> w.contains("SequenceStar")));

        assertEquals(1, spec.models().size());
        assertEquals(NodeKind.ACTION, spec.models().get(0).kind());
        assertEquals(PortType.STRING, spec.models().get(0).ports().get(0).type());
        assertEquals("Value to record", spec.models().get(0).ports().get(0).description());
        assertEquals(9, typed.line());
    }

    @Test
    void singleTreeNeedsNoMainAttribute() {
        TreeSpec spec =
                BtXmlParser.parse(
                        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"A\"><AlwaysSuccess/></BehaviorTree></root>");
        assertEquals("A", spec.mainTreeId());
    }

    @Test
    void multipleTreesWithoutMainLeaveItUnset() {
        TreeSpec spec =
                BtXmlParser.parse(
                        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"A\"><AlwaysSuccess/></BehaviorTree>"
                                + "<BehaviorTree ID=\"B\"><AlwaysSuccess/></BehaviorTree></root>");
        assertEquals("", spec.mainTreeId());
    }

    @Test
    void rejectsScriptingWithLineNumber() {
        BtXmlException e =
                assertThrows(
                        BtXmlException.class,
                        () ->
                                BtXmlParser.parse(
                                        "<root BTCPP_format=\"4\">\n<BehaviorTree ID=\"A\">\n"
                                                + "<AlwaysSuccess _skipIf=\"x\"/>\n</BehaviorTree></root>"));
        assertEquals(3, e.line());
        assertTrue(e.getMessage().contains("_skipIf"));
    }

    @Test
    void structuralErrors() {
        assertThrows(
                BtXmlException.class,
                () ->
                        BtXmlParser.parse(
                                "<root BTCPP_format=\"4\"><BehaviorTree ID=\"A\"><AlwaysSuccess/><AlwaysFailure/></BehaviorTree></root>"),
                "two roots");
        assertThrows(
                BtXmlException.class,
                () -> BtXmlParser.parse("<root BTCPP_format=\"4\"><BehaviorTree ID=\"A\"/></root>"));
        assertThrows(
                BtXmlException.class,
                () -> BtXmlParser.parse("<root BTCPP_format=\"3\"></root>"));
        assertThrows(BtXmlException.class, () -> BtXmlParser.parse("<notroot/>"));
        assertThrows(BtXmlException.class, () -> BtXmlParser.parse("<root><BehaviorTree"));
        assertThrows(
                BtXmlException.class,
                () ->
                        BtXmlParser.parse(
                                "<root BTCPP_format=\"4\" main_tree_to_execute=\"X\"><BehaviorTree ID=\"A\"><AlwaysSuccess/></BehaviorTree></root>"));
    }

    @Test
    void doctypeIsRejected() {
        assertThrows(
                BtXmlException.class,
                () ->
                        BtXmlParser.parse(
                                "<!DOCTYPE root [<!ENTITY x SYSTEM \"file:///etc/passwd\">]><root>&x;</root>"));
    }

    @Test
    void writerRoundTrips() {
        TreeSpec spec = BtXmlParser.parse(SAMPLE);
        String xml = BtXmlWriter.write(spec, spec.models());
        TreeSpec again = BtXmlParser.parse(xml);
        assertEquals(stripLines(spec.trees()), stripLines(again.trees()));
        assertEquals(spec.models(), again.models());
        assertEquals(xml, BtXmlWriter.write(again, again.models()), "writer output is stable");
        assertTrue(xml.endsWith("</root>\n"));
    }

    private static String stripLines(java.util.Map<String, NodeSpec> trees) {
        StringBuilder sb = new StringBuilder();
        trees.forEach((id, n) -> describe(sb.append(id).append(':'), n));
        return sb.toString();
    }

    private static void describe(StringBuilder sb, NodeSpec n) {
        sb.append('(').append(n.id()).append(' ').append(n.name()).append(' ')
                .append(n.subtreeId()).append(' ').append(n.attributes());
        n.children().forEach(c -> describe(sb, c));
        sb.append(')');
    }
}
