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

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.function.Function;

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
                        "<BehaviorTree ID=\"Leg\"><Sequence><Follow trajectory=\"{path}\"/><Follow"
                            + " trajectory=\"Fixed\"/></Sequence></BehaviorTree><BehaviorTree"
                            + " ID=\"Unused\"><Follow trajectory=\"Never\"/></BehaviorTree>");
        assertEquals(
                List.of("P1", "P2", "Fixed", "P3"),
                factory.literalPortValues(BtXmlParser.parse(xml), PortType.TRAJECTORY));
    }

    @Test
    void subtreePortDefaultsNameTrajectoriesAndPointersAreReported() {
        String xml =
                doc(
                        "<Sequence><SubTree ID=\"Leg\"/><SubTree ID=\"Leg\" path=\"P2\"/>"
                                + "<Follow trajectory=\"{tunable}\"/></Sequence>",
                        "<BehaviorTree ID=\"Leg\"><Follow trajectory=\"{path}\"/></BehaviorTree>"
                            + "<TreeNodesModel><SubTree ID=\"Leg\"><input_port name=\"path\""
                            + " default=\"Home\"/></SubTree><SubTree ID=\"Main\"><input_port"
                            + " name=\"tunable\" default=\"P9\"/></SubTree></TreeNodesModel>");
        var spec = BtXmlParser.parse(xml);
        assertEquals(List.of("Home", "P2"), factory.literalPortValues(spec, PortType.TRAJECTORY));
        assertEquals(
                List.of("Follow port 'trajectory' reads {tunable} (in Main)"),
                factory.unresolvedPortValues(spec, PortType.TRAJECTORY),
                "a tree parameter can change at run time, so it can't be preloaded");
    }

    @Test
    void aTrajectoryFromATunableIsALoadError(@TempDir Path dir) throws Exception {
        Path file = dir.resolve("Tuned.xml");
        Files.writeString(
                file,
                "<root BTCPP_format=\"4\" main_tree_to_execute=\"Tuned\"><BehaviorTree"
                    + " ID=\"Tuned\"><Follow"
                    + " trajectory=\"{which}\"/></BehaviorTree><TreeNodesModel><SubTree"
                    + " ID=\"Tuned\"><input_port name=\"which\" type=\"std::string\""
                    + " default=\"P1\"/></SubTree></TreeNodesModel></root>");
        BehaviorTreeAuto auto =
                new BehaviorTreeAuto(
                        factory, file, new com.marswars.bt.core.ParameterStore.InMemory());
        assertFalse(auto.reload());
        assertFalse(auto.isLoaded());
        assertTrue(auto.getError().contains("reads {which}"), auto.getError());
    }

    @Test
    void listXmlSortsAndFilters(@TempDir Path dir) throws Exception {
        Files.writeString(dir.resolve("b.xml"), "x");
        Files.writeString(dir.resolve("a.xml"), "x");
        Files.writeString(dir.resolve("notes.txt"), "x");
        Files.createDirectory(dir.resolve("sub.xml"));
        assertEquals(List.of(dir.resolve("a.xml"), dir.resolve("b.xml")), BtXmlParser.listXml(dir));
        assertEquals(List.of(), BtXmlParser.listXml(dir.resolve("missing")));
    }

    @Test
    void libraryFilesAreNotAutos(@TempDir Path dir) throws Exception {
        Files.writeString(dir.resolve("Auto.xml"), doc("<AlwaysSuccess/>", ""));
        Files.writeString(
                dir.resolve("common.xml"),
                "<root BTCPP_format=\"4\"><BehaviorTree ID=\"A\"><AlwaysSuccess/></BehaviorTree>"
                        + "<BehaviorTree ID=\"B\"><AlwaysSuccess/></BehaviorTree></root>");
        // one tree but no main_tree_to_execute: still a library, an auto must name its main tree
        Files.writeString(
                dir.resolve("single.xml"),
                "<root BTCPP_format=\"4\"><BehaviorTree"
                    + " ID=\"Only\"><AlwaysSuccess/></BehaviorTree></root>");
        Files.writeString(dir.resolve("Broken.xml"), "<root BTCPP_format=\"4\"><BehaviorTree");
        assertEquals(
                List.of(dir.resolve("Auto.xml"), dir.resolve("Broken.xml")),
                BehaviorTreeAuto.listAutos(dir));
        assertEquals(
                List.of("Auto", "Broken"),
                BehaviorTreeAuto.loadAll(factory, dir).stream().map(a -> a.getName()).toList());
    }

    @Test
    void loadsReloadsAndUnloadsOnFailure(@TempDir Path dir) throws Exception {
        Path file = dir.resolve("MyAuto.xml");
        Files.writeString(file, doc("<Follow trajectory=\"P1\"/>", ""));
        BehaviorTreeAuto auto = new BehaviorTreeAuto(factory, file);
        assertFalse(auto.isLoaded(), "constructing reads nothing");
        assertTrue(auto.reload());

        assertEquals("MyAuto", auto.getName());
        assertTrue(auto.isLoaded());
        assertEquals("", auto.getError());
        assertEquals(List.of("P1"), auto.getTrajectoryNames());

        Files.writeString(
                file,
                doc(
                        "<Sequence><Follow trajectory=\"P2\"/><Follow"
                            + " trajectory=\"P3\"/></Sequence>",
                        ""));
        assertTrue(auto.reload());
        assertEquals(List.of("P2", "P3"), auto.getTrajectoryNames());

        Files.writeString(file, doc("<Follow trajectory=\"P9\" bogus=\"1\"/>", ""));
        assertFalse(auto.reload());
        assertTrue(auto.getError().contains("unknown port 'bogus'"), auto.getError());
        assertFalse(auto.isLoaded(), "a failed load unloads the auto");
        assertEquals(List.of(), auto.getTrajectoryNames());
        assertThrows(BtException.class, auto::buildTree);

        // A pointer nothing sets (a typo in a tunable's name) is a load error too, not a null
        // mid-auto; the auto's own seeded root keys ({@auto}) are fine.
        Files.writeString(file, doc("<Follow trajectory=\"P2\" speed=\"{sped}\"/>", ""));
        assertFalse(auto.reload());
        assertTrue(auto.getError().contains("reads {sped}, which nothing sets"), auto.getError());
        Files.writeString(file, doc("<Follow trajectory=\"P2\" speed=\"{@auto}\"/>", ""));
        assertTrue(auto.reload(), auto.getError());

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
        autos.forEach(BehaviorTreeAuto::reload);
        BehaviorTreeAuto broken = autos.get(0);
        assertFalse(broken.isLoaded());
        assertFalse(broken.getError().isEmpty());
        assertThrows(BtException.class, broken::buildTree);
        assertTrue(autos.get(1).isLoaded());
        assertTrue(messages.stream().anyMatch(m -> m.contains("Broken")));
    }

    @Test
    void treeParametersComeFromStoreEachRun(@TempDir Path dir) throws Exception {
        Path file = dir.resolve("Params.xml");
        Files.writeString(
                file,
                "<root BTCPP_format=\"4\" main_tree_to_execute=\"Params\">"
                        + "<BehaviorTree ID=\"Params\"><Delay delay_msec=\"{wait_msec}\">"
                        + "<AlwaysSuccess/></Delay></BehaviorTree>"
                        + "<TreeNodesModel><SubTree ID=\"Params\">"
                        + "<input_port name=\"wait_msec\" type=\"int\" default=\"3000\"/>"
                        + "</SubTree></TreeNodesModel></root>");
        com.marswars.bt.core.ParameterStore.InMemory store =
                new com.marswars.bt.core.ParameterStore.InMemory();
        BehaviorTreeAuto auto = new BehaviorTreeAuto(factory, file, store);
        auto.reload();

        assertEquals("Autos/Params/wait_msec", auto.parameterKey("wait_msec"));
        assertEquals(
                java.util.Map.of("Autos/Params/wait_msec", 3000),
                store.snapshot(),
                "registered at load so it can be tuned while disabled");
        assertEquals(3000, auto.buildTree().getBlackboard().get("wait_msec"));

        store.set("Autos/Params/wait_msec", 0);
        BehaviorTree tree = auto.buildTree();
        assertEquals(0, tree.getBlackboard().get("wait_msec"));
        assertEquals(NodeStatus.SUCCESS, tree.tickOnce(), "Delay 0 runs its child at once");
    }

    @Test
    void trajectoryResolverRequiresAuto() {
        BehaviorTree tree = factory.createTreeFromText(doc("<AlwaysSuccess/>", ""));
        assertThrows(BtException.class, () -> BehaviorTreeAuto.trajectories(tree.getRoot()));
    }
}
