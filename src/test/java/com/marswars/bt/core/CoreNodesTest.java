package com.marswars.bt.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.marswars.bt.control.SequenceNode;
import com.marswars.bt.decorator.InverterNode;
import com.marswars.bt.support.BtTestBase;
import com.marswars.bt.support.ScriptedAction;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class CoreNodesTest extends BtTestBase {

    @Test
    void haltOfNeverStartedStatefulNodeSkipsOnHalted() {
        ScriptedAction a = action("a", R);
        a.haltNode();
        assertEquals(0, a.halts);
        a.executeTick();
        a.haltNode();
        assertEquals(1, a.halts);
        assertEquals(I, a.getStatus());
    }

    @Test
    void completedStatefulNodeKeepsStatusUntilReset() {
        ScriptedAction a = action("a", S);
        assertEquals(S, a.executeTick());
        assertEquals(S, a.executeTick());
        assertEquals(1, a.starts);
        a.resetStatus();
        a.executeTick();
        assertEquals(2, a.starts);
    }

    @Test
    void idleResultThrows() {
        ScriptedAction a = action("a", I);
        assertThrows(BtException.class, a::executeTick);
    }

    @Test
    void syncActionAndConditionRejectRunning() {
        SimpleActionNode sync = new SimpleActionNode("s", cfg(), n -> NodeStatus.RUNNING);
        assertThrows(BtException.class, sync::executeTick);
    }

    @Test
    void portsResolveLiteralsPointersAndDefaults() {
        NodeModel model =
                new NodeModel(
                        "Thing",
                        NodeKind.ACTION,
                        List.of(
                                PortInfo.input("speed", PortType.DOUBLE, "1.5", ""),
                                PortInfo.input("count", PortType.INT, ""),
                                PortInfo.output("out", PortType.INT, "")),
                        "",
                        NodeOrigin.ROBOT);
        bb.set("n", 7);
        NodeConfig c =
                cfg().withModel(model)
                        .withPorts(Map.of("count", "{n}"), Map.of("out", "{result}"));
        SimpleActionNode node =
                new SimpleActionNode(
                        null,
                        c,
                        n -> {
                            n.setOutput("out", n.getInt("count") * 2);
                            return NodeStatus.SUCCESS;
                        });
        assertEquals("Thing", node.getName(), "name defaults to registration id");
        assertEquals(1.5, node.getDouble("speed"));
        assertEquals(7, node.getInt("count"));
        node.executeTick();
        assertEquals(14, bb.get("result"));
    }

    @Test
    void outputPortAcceptsPointerPlainKeyAndShorthand() {
        for (String raw : new String[] {"{dest}", "dest"}) {
            NodeConfig c = cfg().withPorts(Map.of(), Map.of("out", raw));
            new SimpleActionNode(
                            "x",
                            c,
                            n -> {
                                n.setOutput("out", raw);
                                return NodeStatus.SUCCESS;
                            })
                    .executeTick();
            assertEquals(raw, bb.get("dest"));
        }
        NodeConfig same = cfg().withPorts(Map.of(), Map.of("out", "{=}"));
        new SimpleActionNode(
                        "y",
                        same,
                        n -> {
                            n.setOutput("out", 5);
                            return NodeStatus.SUCCESS;
                        })
                .executeTick();
        assertEquals(5, bb.get("out"));

        SimpleActionNode unset =
                new SimpleActionNode(
                        "z",
                        cfg(),
                        n -> {
                            n.setOutput("missing", 1);
                            return NodeStatus.SUCCESS;
                        });
        assertThrows(BtException.class, unset::executeTick);
    }

    @Test
    void enumPortParsing() {
        SimpleActionNode node =
                new SimpleActionNode("x", cfg(Map.of("s", "RUNNING")), n -> NodeStatus.SUCCESS);
        assertEquals(NodeStatus.RUNNING, node.getEnum("s", NodeStatus.class));
        SimpleActionNode bad =
                new SimpleActionNode("y", cfg(Map.of("s", "NOPE")), n -> NodeStatus.SUCCESS);
        assertThrows(BtException.class, () -> bad.getEnum("s", NodeStatus.class));
    }

    @Test
    void treeAssignsPreorderUidsAndStatusString() {
        ScriptedAction a = action("a", S);
        ScriptedAction b = action("b", R);
        InverterNode inv = with(new InverterNode("inv", cfg()), b);
        SequenceNode root = with(new SequenceNode("root", cfg()), a, inv);
        BehaviorTree tree = tree(root);

        assertEquals(List.of(root, a, inv, b), tree.getNodes());
        assertEquals(1, root.getUid());
        assertEquals(4, b.getUid());
        assertEquals("IIII", tree.statusString());
        assertEquals(R, tree.tickOnce());
        assertEquals("RSRR", tree.statusString());

        tree.haltTree();
        assertEquals("IIII", tree.statusString());
        assertEquals(1, b.halts);
    }

    @Test
    void completedRootIsResetOnNextTick() {
        ScriptedAction a = action("a", S);
        BehaviorTree tree = tree(with(new SequenceNode("root", cfg()), a));
        assertEquals(S, tree.tickOnce());
        assertEquals(S, tree.getRoot().getStatus());
        assertEquals(S, tree.tickOnce());
        assertEquals(2, a.starts);
    }

    @Test
    void listenerSeesTransitionsIncludingIdle() {
        ScriptedAction a = action("a", R, S);
        BehaviorTree tree = tree(with(new SequenceNode("root", cfg()), a));
        List<String> events = new ArrayList<>();
        tree.setStatusListener(
                (node, prev, next, t) -> events.add(node.getName() + ":" + prev + ">" + next));
        tree.tickOnce();
        tree.tickOnce();
        assertEquals(
                List.of(
                        "root:IDLE>RUNNING",
                        "a:IDLE>RUNNING",
                        "a:RUNNING>SUCCESS",
                        "a:SUCCESS>IDLE",
                        "root:RUNNING>SUCCESS"),
                events);
    }

    @Test
    void blackboardScopingRules() {
        Blackboard root = Blackboard.createRoot();
        root.set("shared", 1);
        root.set("goal", "A");

        Blackboard child = Blackboard.createChild(root);
        assertNull(child.get("shared"), "no implicit inheritance");
        child.addSubtreeRemapping("target", "goal");
        assertEquals("A", child.get("target"));
        child.set("target", "B");
        assertEquals("B", root.get("goal"), "remapped writes go to the parent");

        child.setLocal("literal", "x");
        assertNull(root.get("literal"));

        Blackboard auto = Blackboard.createChild(root);
        auto.enableAutoRemapping(true);
        assertEquals(1, auto.get("shared"));
        auto.set("shared", 2);
        assertEquals(2, root.get("shared"));
        auto.set("fresh", 3);
        assertNull(root.get("fresh"), "new keys stay local");

        Blackboard deep = Blackboard.createChild(child);
        deep.set("@rootkey", 9);
        assertEquals(9, root.get("rootkey"));
        assertEquals(9, deep.get("@rootkey"));
    }
}
