package com.marswars.bt.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.bt.support.FakeClock;
import com.marswars.bt.xml.BtXmlParser;
import com.marswars.bt.xml.BtXmlParserTestAccess;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class BehaviorTreeFactoryTest {
    enum Mode {
        STORE,
        INTAKE
    }

    final FakeClock clock = new FakeClock();
    final List<String> recorded = new ArrayList<>();
    final List<Mode> modes = new ArrayList<>();
    BehaviorTreeFactory factory;

    @BeforeEach
    void setUp() {
        factory = new BehaviorTreeFactory(clock);
        factory.registerInstantAction(
                "RecordInput",
                "Records its input",
                List.of(PortInfo.input("value", PortType.STRING, "Value to record")),
                n -> recorded.add(n.getString("value")));
        factory.registerSetState("SetMode", "Sets the mode", Mode.class, modes::add);
        factory.registerSimpleCondition(
                "IsEven",
                "",
                List.of(PortInfo.input("n", PortType.INT, "number")),
                n -> n.getInt("n") % 2 == 0);
    }

    private static String doc(String body) {
        return "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">" + body + "</BehaviorTree></root>";
    }

    @Test
    void sampleRunsWithSubtreeRemappingAndPaths() {
        BehaviorTree tree = factory.createTreeFromFile(BtXmlParserTestAccess.SAMPLE);
        assertEquals(NodeStatus.SUCCESS, tree.tickOnce());
        assertEquals(List.of("7", "7", "literal"), recorded);
        BehaviorTree.Subtree inner = tree.getSubtrees().get(1);
        BehaviorTree.Subtree shared = tree.getSubtrees().get(2);
        assertEquals("literal", inner.blackboard().get("label"), "literal stays local");
        assertEquals(null, tree.getBlackboard().get("label"));
        assertEquals("99", shared.blackboard().get("local_copy"),
                "_autoremap reads the parent's speed after Inner wrote 99 through {target}");
        assertEquals(null, tree.getBlackboard().get("local_copy"), "new keys stay in the subtree");
        assertEquals("from_shared", tree.getBlackboard().get("speed"),
                "_autoremap writes an existing parent key");

        List<String> paths = new ArrayList<>();
        for (TreeNode n : tree.getNodes()) {
            paths.add(n.getUid() + ":" + n.getPath());
        }
        assertEquals(
                List.of(
                        "1:root",
                        "2:SetBlackboard::2",
                        "3:RecordInput::3",
                        "4:Inner::4",
                        "5:Inner::4/Sequence::5",
                        "6:Inner::4/RecordInput::6",
                        "7:Inner::4/RecordInput::7",
                        "8:Inner::4/SetBlackboard::8",
                        "9:Shared::9",
                        "10:Shared::9/Sequence::10",
                        "11:Shared::9/SetBlackboard::11",
                        "12:Shared::9/SetBlackboard::12",
                        "13:SequenceWithMemory::13",
                        "14:AlwaysSuccess::14"),
                paths);
        assertEquals(3, tree.getSubtrees().size());
        assertEquals("Inner::4", inner.instanceName());
        assertEquals("Inner", inner.treeId());
    }

    @Test
    void setStateParsesEnum() {
        BehaviorTree tree =
                factory.createTreeFromText(
                        doc("<Sequence><SetMode state=\"INTAKE\"/><SetMode state=\"STORE\"/></Sequence>"));
        tree.tickOnce();
        assertEquals(List.of(Mode.INTAKE, Mode.STORE), modes);
        assertEquals(List.of("STORE", "INTAKE"),
                factory.getNodeModel("SetMode").orElseThrow().ports().get(0).choices());
    }

    @Test
    void conditionWithPointerPort() {
        BehaviorTree tree =
                factory.createTreeFromText(
                        doc("<Sequence><SetBlackboard value=\"4\" output_key=\"x\"/><IsEven n=\"{x}\"/></Sequence>"));
        assertEquals(NodeStatus.SUCCESS, tree.tickOnce());
    }

    @Test
    void validationErrors() {
        assertError("unknown node ID 'Nope'", doc("<Nope/>"));
        assertError("not supported by MW-Lib", doc("<Script code=\"x:=1\"/>"));
        assertError("unknown port 'bogus'", doc("<RecordInput value=\"a\" bogus=\"1\"/>"));
        assertError("missing required port 'value'", doc("<RecordInput/>"));
        assertError("expected one of", doc("<SetMode state=\"FLY\"/>"));
        assertError("as Integer", doc("<IsEven n=\"abc\"/>"));
        assertError("cannot have children", doc("<RecordInput value=\"a\"><AlwaysSuccess/></RecordInput>"));
        assertError("exactly one child", doc("<Inverter><AlwaysSuccess/><AlwaysSuccess/></Inverter>"));
        assertError("at least one child", doc("<Sequence></Sequence>"));
        assertError("2 or 3 children", doc("<IfThenElse><AlwaysSuccess/></IfThenElse>"));
        assertError("success_count 3 > 2", doc("<Parallel success_count=\"3\"><AlwaysSuccess/><AlwaysSuccess/></Parallel>"));
        assertError("is not a defined", doc("<SubTree ID=\"Missing\"/>"));
        assertError(
                "recursive SubTree",
                "<root BTCPP_format=\"4\" main_tree_to_execute=\"A\">"
                        + "<BehaviorTree ID=\"A\"><SubTree ID=\"B\"/></BehaviorTree>"
                        + "<BehaviorTree ID=\"B\"><SubTree ID=\"A\"/></BehaviorTree></root>");
        assertError(
                "main_tree_to_execute is not set",
                "<root BTCPP_format=\"4\"><BehaviorTree ID=\"A\"><AlwaysSuccess/></BehaviorTree>"
                        + "<BehaviorTree ID=\"B\"><AlwaysSuccess/></BehaviorTree></root>");
    }

    /** The blackboard-flow check (separate from createTree): every {key} an input reads has a
     * source. */
    @Test
    void blackboardPointersMustHaveASource() {
        assertFlowError("reads {nope}, which nothing sets", doc("<RecordInput value=\"{nope}\"/>"));
        assertFlowError("reads {typo}, which nothing sets", doc("<IsEven n=\"{typo}\"/>"));
        // createTree itself stays permissive: code may fill the blackboard after building
        factory.createTreeFromText(doc("<RecordInput value=\"{nope}\"/>"));

        String params = "<TreeNodesModel><SubTree ID=\"Main\">"
                + "<input_port name=\"p\" type=\"std::string\" default=\"a\"/>"
                + "<input_port name=\"q\" type=\"std::string\"/>"
                + "</SubTree><SubTree ID=\"Sub\">"
                + "<input_port name=\"v\" type=\"std::string\"/>"
                + "<input_port name=\"d\" type=\"std::string\" default=\"x\"/>"
                + "<output_port name=\"result\" type=\"std::string\"/>"
                + "</SubTree></TreeNodesModel>";
        String sub = "<BehaviorTree ID=\"Sub\"><Sequence><RecordInput value=\"{v}\"/>"
                + "<RecordInput value=\"{d}\"/><SetBlackboard output_key=\"{result}\" value=\"r\"/>"
                + "</Sequence></BehaviorTree>";
        java.util.function.Function<String, String> main =
                body -> "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\">"
                        + "<BehaviorTree ID=\"Main\">" + body + "</BehaviorTree>" + sub + params + "</root>";

        // a tree parameter with a default, a value written earlier by SetBlackboard, a port passed
        // to a subtree, a subtree output mapped back, and _autoremap all count as sources
        checkFlow(main.apply("<RecordInput value=\"{p}\"/>"));
        checkFlow(main.apply(
                "<Sequence><SetBlackboard output_key=\"{x}\" value=\"1\"/><RecordInput value=\"{x}\"/></Sequence>"));
        checkFlow(main.apply(
                "<Sequence><SubTree ID=\"Sub\" v=\"{p}\" result=\"{r}\"/><RecordInput value=\"{r}\"/></Sequence>"));
        checkFlow(main.apply("<SubTree ID=\"Sub\" v=\"literal\"/>"));
        checkFlow(main.apply(
                "<Sequence><SetBlackboard output_key=\"{v}\" value=\"1\"/><SubTree ID=\"Sub\" _autoremap=\"true\"/></Sequence>"));
        // a Repeat may read on its second pass what a later sibling wrote on the first
        checkFlow(main.apply(
                "<Repeat num_cycles=\"2\"><Sequence><RecordInput value=\"{p}\"/>"
                        + "<SetBlackboard output_key=\"{later}\" value=\"1\"/><RecordInput value=\"{later}\"/></Sequence></Repeat>"));
        // a root key the robot code pre-populated
        Blackboard root = Blackboard.createRoot();
        root.set("seeded", "yes");
        factory.checkBlackboardFlow(BtXmlParser.parse(main.apply("<RecordInput value=\"{@seeded}\"/>")), root);

        // a tree parameter without a default is never set
        assertFlowError("reads {q}, which nothing sets", main.apply("<RecordInput value=\"{q}\"/>"));
        // a subtree port the caller didn't pass (and that has no default)
        assertFlowError("reads {v}, which nothing sets", main.apply("<SubTree ID=\"Sub\"/>"));
        // passing a pointer the caller itself doesn't have
        assertFlowError("reads {missing}, which nothing sets", main.apply("<SubTree ID=\"Sub\" v=\"{missing}\"/>"));
        // a root key nobody seeded
        assertFlowError("reads {@nobody}, which nothing sets", main.apply("<RecordInput value=\"{@nobody}\"/>"));
    }

    private void checkFlow(String xml) {
        factory.checkBlackboardFlow(BtXmlParser.parse(xml), Blackboard.createRoot());
    }

    private void assertFlowError(String expected, String xml) {
        BtException e = assertThrows(BtException.class, () -> checkFlow(xml));
        assertTrue(
                e.getMessage().contains(expected),
                "expected '" + expected + "' in: " + e.getMessage());
    }

    private void assertError(String expected, String xml) {
        BtException e = assertThrows(BtException.class, () -> factory.createTreeFromText(xml));
        assertTrue(
                e.getMessage().contains(expected),
                "expected '" + expected + "' in: " + e.getMessage());
    }

    @Test
    void duplicateRegistrationThrows() {
        assertThrows(
                BtException.class,
                () -> factory.registerInstantAction("RecordInput", "", List.of(), n -> {}));
        assertThrows(
                BtException.class,
                () -> factory.registerInstantAction("Sequence", "", List.of(), n -> {}));
    }

    @Test
    void nodeSpecXmlMatchesBtcppFormatAndRoundTrips() {
        String xml = factory.writeTreeNodesModelXml(false);
        assertTrue(xml.contains("<root BTCPP_format=\"4\">"));
        assertTrue(xml.contains("<Action ID=\"RecordInput\">"));
        assertTrue(xml.contains("<input_port name=\"value\" type=\"std::string\">Value to record</input_port>"));
        assertTrue(xml.contains("<input_port name=\"state\" type=\"Mode\">State to request</input_port>"));
        assertTrue(xml.contains("<Condition ID=\"IsEven\">"));
        assertTrue(!xml.contains("ID=\"Sequence\""), "built-ins excluded");
        assertTrue(factory.writeTreeNodesModelXml(true).contains("<Control ID=\"Parallel\">"));
        // the node-spec document is itself parseable as a (tree-less) BT.CPP file fragment
        String withTree = xml.replace("<root BTCPP_format=\"4\">",
                "<root BTCPP_format=\"4\"><BehaviorTree ID=\"X\"><AlwaysSuccess/></BehaviorTree>");
        assertEquals(4, BtXmlParser.parse(withTree).models().size(), "ParallelDeadline + 3 robot");
        int lib = xml.indexOf("<!-- MW-Lib shared nodes (com.marswars.bt) -->");
        int robot = xml.indexOf("<!-- Robot nodes -->");
        assertTrue(lib > 0 && robot > lib, "robot nodes are appended after MW-Lib's");
        assertTrue(xml.indexOf("ID=\"ParallelDeadline\"") < robot);
        assertTrue(xml.indexOf("ID=\"RecordInput\"") > robot);
    }

    @Test
    void modelsJsonHasChoicesAndDefaults() {
        String json = factory.nodeModelsJson();
        assertTrue(json.contains("\"id\":\"SetMode\""));
        assertTrue(json.contains("\"choices\":[\"STORE\",\"INTAKE\"]"));
        assertTrue(json.contains("\"name\":\"success_count\",\"direction\":\"input\",\"type\":\"int\",\"default\":\"-1\""));
    }

    @Test
    void modelsUsedByListsOnlyCustomNodes() {
        var spec = BtXmlParser.parse(doc("<Sequence><SetMode state=\"STORE\"/><AlwaysSuccess/></Sequence>"));
        assertEquals(List.of("SetMode"),
                factory.modelsUsedBy(spec).stream().map(NodeModel::id).toList());
    }

    @Test
    void builderMatchesXml() {
        TreeSpec built =
                TreeBuilder.create("Main")
                        .begin("Sequence", "name", "seq")
                        .node("SetMode", "state", "INTAKE")
                        .begin("Parallel", "success_count", "1")
                        .node("Sleep", "msec", "1000")
                        .subTree("Sub", "x", "{y}")
                        .end()
                        .end()
                        .tree("Sub")
                        .node("AlwaysSuccess")
                        .build();
        TreeSpec parsed =
                BtXmlParser.parse(
                        "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\">"
                                + "<BehaviorTree ID=\"Main\"><Sequence name=\"seq\">"
                                + "<SetMode state=\"INTAKE\"/><Parallel success_count=\"1\">"
                                + "<Sleep msec=\"1000\"/><SubTree ID=\"Sub\" x=\"{y}\"/></Parallel>"
                                + "</Sequence></BehaviorTree>"
                                + "<BehaviorTree ID=\"Sub\"><AlwaysSuccess/></BehaviorTree></root>");
        assertEquals(
                com.marswars.bt.xml.BtXmlWriter.write(parsed, List.of()),
                com.marswars.bt.xml.BtXmlWriter.write(built, List.of()));
        BehaviorTree tree = factory.createTree(built);
        assertEquals(6, tree.getNodes().size());
    }

    @Test
    void registeredTreesServeAsSubtreeLibrary() {
        factory.registerBehaviorTreeFromText(
                "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Lib\"><SetMode state=\"STORE\"/></BehaviorTree></root>");
        BehaviorTree tree = factory.createTreeFromText(doc("<SubTree ID=\"Lib\"/>"));
        tree.tickOnce();
        assertEquals(List.of(Mode.STORE), modes);
        assertEquals(NodeStatus.SUCCESS, factory.createTree("Lib").tickOnce());
    }

    @Test
    void createTreeFromFileKeepsXml() {
        BehaviorTree tree = factory.createTreeFromFile(BtXmlParserTestAccess.SAMPLE);
        assertTrue(tree.getXml().orElseThrow().contains("<BehaviorTree ID=\"Inner\">"));
        assertEquals("MainTree", tree.getMainTreeId());
    }
}
