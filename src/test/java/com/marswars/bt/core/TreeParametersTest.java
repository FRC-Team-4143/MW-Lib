package com.marswars.bt.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import com.marswars.bt.support.FakeClock;
import com.marswars.bt.xml.BtXmlParser;
import java.util.List;
import org.junit.jupiter.api.Test;

/** Tree ports declared in {@code <TreeNodesModel>} as {@code <SubTree ID=...>} models. */
class TreeParametersTest {
    static final String XML =
            "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\">"
                    + "<BehaviorTree ID=\"Main\"><Sequence>"
                    + "<Delay delay_msec=\"{wait_msec}\"><AlwaysSuccess/></Delay>"
                    + "<SubTree ID=\"Child\" given=\"{wait_msec}\"/>"
                    + "</Sequence></BehaviorTree>"
                    + "<BehaviorTree ID=\"Child\"><AlwaysSuccess/></BehaviorTree>"
                    + "<TreeNodesModel>"
                    + "<SubTree ID=\"Main\">"
                    + "<input_port name=\"wait_msec\" type=\"int\" default=\"500\">Delay</input_port>"
                    + "<input_port name=\"label\" type=\"std::string\" default=\"hi\"/>"
                    + "<input_port name=\"required_thing\" type=\"double\"/>"
                    + "</SubTree>"
                    + "<SubTree ID=\"Child\">"
                    + "<input_port name=\"given\" type=\"int\" default=\"1\"/>"
                    + "<input_port name=\"speed\" type=\"double\" default=\"2.5\"/>"
                    + "</SubTree>"
                    + "</TreeNodesModel></root>";

    final FakeClock clock = new FakeClock();

    @Test
    void mainTreeDefaultsSeedRootBlackboard() {
        BehaviorTree tree = new BehaviorTreeFactory(clock).createTreeFromText(XML);
        assertEquals(500, tree.getBlackboard().get("wait_msec"));
        assertEquals("hi", tree.getBlackboard().get("label"));
        assertNull(tree.getBlackboard().get("required_thing"), "no default, nothing seeded");

        assertEquals(NodeStatus.RUNNING, tree.tickOnce());
        clock.advance(0.49);
        assertEquals(NodeStatus.RUNNING, tree.tickOnce());
        clock.advance(0.02);
        assertEquals(NodeStatus.SUCCESS, tree.tickOnce());
    }

    @Test
    void callerValuesWinOverDefaults() {
        BtXmlParser.parse(XML);
        Blackboard root = Blackboard.createRoot();
        root.set("wait_msec", 0);
        BehaviorTree tree =
                new BehaviorTreeFactory(clock).createTree(BtXmlParser.parse(XML), root, XML);
        assertEquals(0, tree.getBlackboard().get("wait_msec"));
        assertEquals(NodeStatus.SUCCESS, tree.tickOnce());
    }

    @Test
    void subTreePortDefaultsApplyWhenNotRemapped() {
        BehaviorTree tree = new BehaviorTreeFactory(clock).createTreeFromText(XML);
        Blackboard child = tree.getSubtrees().get(1).blackboard();
        assertEquals(500, child.get("given"), "remapped port reads the parent entry");
        assertEquals(2.5, child.get("speed"), "unset port takes the model default");
    }

    @Test
    void mainTreeParametersListed() {
        List<String> names =
                BtXmlParser.parse(XML).mainTreeParameters().stream().map(PortInfo::name).toList();
        assertEquals(List.of("wait_msec", "label", "required_thing"), names);
    }

    @Test
    void typedLiterals() {
        assertEquals(3, ParameterStore.typed(PortInfo.input("a", PortType.INT, ""), "3"));
        assertEquals(1.5, ParameterStore.typed(PortInfo.input("a", PortType.DOUBLE, ""), "1.5"));
        assertEquals(true, ParameterStore.typed(PortInfo.input("a", PortType.BOOLEAN, ""), "true"));
        assertEquals("x", ParameterStore.typed(PortInfo.input("a", PortType.ANY, ""), "x"));
    }
}
