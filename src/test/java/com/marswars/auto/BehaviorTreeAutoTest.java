package com.marswars.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.BtException;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.PortType;
import com.marswars.bt.xml.BtXmlParser;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.function.Function;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class BehaviorTreeAutoTest {
    BehaviorTreeFactory factory;
    final List<String> messages = new java.util.ArrayList<>();

    @org.junit.jupiter.api.AfterEach
    void restoreLog() {
        BehaviorTreeAuto.message_log_ = edu.wpi.first.wpilibj.DataLogManager::log;
    }

    @BeforeEach
    void setUp() {
        BehaviorTreeAuto.message_log_ = messages::add;
        factory = new BehaviorTreeFactory(() -> 0.0);
        factory.registerInstantAction(
                "Follow",
                "",
                List.of(
                        PortInfo.trajectory("trajectory", "path"),
                        PortInfo.input("speed", PortType.DOUBLE, "1", "")),
                n -> {});
    }

    static String doc(String main, String extraTrees) {
        return "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\">"
                + main
                + "</BehaviorTree>"
                + extraTrees
                + "</root>";
    }

    @Test
    void discoversLiteralTrajectoriesInOrderThroughSubtrees() {
        String xml =
                doc(
                        "<Sequence><Follow trajectory=\"P1\"/><Follow trajectory=\"{dyn}\"/>"
                                + "<SubTree ID=\"Leg\" path=\"P2\"/><Follow trajectory=\"P1\"/>"
                                + "<SubTree ID=\"Leg\" path=\"{unknown}\"/>"
                                + "<SubTree ID=\"Leg\" path=\"P3\"/></Sequence>",
                        "<BehaviorTree ID=\"Leg\"><Sequence><Follow trajectory=\"{path}\"/>"
                                + "<Follow trajectory=\"Fixed\"/></Sequence></BehaviorTree>"
                                + "<BehaviorTree ID=\"Unused\"><Follow trajectory=\"Never\"/></BehaviorTree>");
        assertEquals(
                List.of("P1", "P2", "Fixed", "P3"),
                BehaviorTreeAuto.discoverTrajectoryNames(BtXmlParser.parse(xml), factory));
    }

    @Test
    void listXmlSortsAndFilters(@TempDir Path dir) throws Exception {
        Files.writeString(dir.resolve("b.xml"), "x");
        Files.writeString(dir.resolve("a.xml"), "x");
        Files.writeString(dir.resolve("notes.txt"), "x");
        Files.createDirectory(dir.resolve("sub.xml"));
        assertEquals(
                List.of(dir.resolve("a.xml"), dir.resolve("b.xml")), BehaviorTreeAuto.listXml(dir));
        assertEquals(List.of(), BehaviorTreeAuto.listXml(dir.resolve("missing")));
    }

    @Test
    void loadsReloadsAndKeepsLastGoodTree(@TempDir Path dir) throws Exception {
        Path file = dir.resolve("MyAuto.xml");
        Files.writeString(file, doc("<Follow trajectory=\"P1\"/>", ""));
        BehaviorTreeAuto auto = new BehaviorTreeAuto(factory, file);

        assertEquals("MyAuto", auto.getName());
        assertTrue(auto.isLoaded());
        assertEquals("", auto.getError());
        assertEquals(List.of("P1"), auto.getTrajectoryNames());

        Files.writeString(file, doc("<Sequence><Follow trajectory=\"P2\"/><Follow trajectory=\"P3\"/></Sequence>", ""));
        assertTrue(auto.reload());
        assertEquals(List.of("P2", "P3"), auto.getTrajectoryNames());

        Files.writeString(file, doc("<Follow trajectory=\"P9\" bogus=\"1\"/>", ""));
        assertFalse(auto.reload());
        assertTrue(auto.getError().contains("unknown port 'bogus'"), auto.getError());
        assertEquals(List.of("P2", "P3"), auto.getTrajectoryNames(), "last good tree kept");

        BehaviorTree tree = auto.buildTree();
        assertEquals("MyAuto", tree.getBlackboard().get("auto"));
        assertTrue(tree.getBlackboard().get("trajectories") instanceof Function<?, ?>);
        assertEquals(NodeStatus.SUCCESS, tree.tickOnce());
    }

    @Test
    void brokenFileStillRegistersWithError(@TempDir Path dir) throws Exception {
        Files.writeString(dir.resolve("Good.xml"), doc("<AlwaysSuccess/>", ""));
        Files.writeString(dir.resolve("Broken.xml"), "<root BTCPP_format=\"4\"><BehaviorTree");
        List<BehaviorTreeAuto> autos = BehaviorTreeAuto.loadAll(factory, dir);
        assertEquals(List.of("Broken", "Good"), autos.stream().map(a -> a.getName()).toList());
        BehaviorTreeAuto broken = autos.get(0);
        assertFalse(broken.isLoaded());
        assertFalse(broken.getError().isEmpty());
        assertThrows(BtException.class, broken::buildTree);
        assertTrue(autos.get(1).isLoaded());
        assertTrue(messages.stream().anyMatch(m -> m.contains("Broken")));
    }

    @Test
    void trajectoryResolverRequiresAuto() {
        BehaviorTree tree = factory.createTreeFromText(doc("<AlwaysSuccess/>", ""));
        assertThrows(
                BtException.class, () -> BehaviorTreeAuto.trajectories(tree.getRoot()));
    }
}
