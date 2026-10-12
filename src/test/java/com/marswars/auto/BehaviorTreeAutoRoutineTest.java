package com.marswars.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.NodeKind;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.ParameterStore;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.StatefulActionNode;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** A behavior-tree auto as an AutoRoutine: run from the robot loop, prepared in two halves. */
class BehaviorTreeAutoRoutineTest {
    final List<String> messages = new ArrayList<>();
    final List<String> ran = new ArrayList<>();
    BehaviorTreeFactory factory;
    final AtomicReference<String> xml = new AtomicReference<>();

    static String doc(String body) {
        return "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\">"
                + body
                + "</BehaviorTree></root>";
    }

    @BeforeEach
    void setUp() {
        BehaviorTreeAuto.message_log_ = messages::add;
        factory = new BehaviorTreeFactory(() -> 0.0);
        // RUNNING for one tick, then SUCCESS
        factory.registerNodeType(
                "Step",
                NodeKind.ACTION,
                "",
                List.of(PortInfo.input("id", com.marswars.bt.core.PortType.STRING, "")),
                (name, cfg) ->
                        new StatefulActionNode(name, cfg) {
                            @Override
                            protected NodeStatus onStart() {
                                ran.add(cfg.inputPorts().get("id"));
                                return NodeStatus.RUNNING;
                            }

                            @Override
                            protected NodeStatus onRunning() {
                                return NodeStatus.SUCCESS;
                            }

                            @Override
                            protected void onHalted() {
                                ran.add("halted " + cfg.inputPorts().get("id"));
                            }
                        });
        factory.registerInstantAction("Follow", "", List.of(PortInfo.trajectory("trajectory", "path")), n -> {});
        factory.registerInstantAction("WaitEvent", "", List.of(PortInfo.event("event", "")), n -> {});
    }

    /** A three-sample swerve path with one event marker. */
    static choreo.trajectory.Trajectory<?> pathWithEvent(String name, String event) {
        var samples =
                List.of(
                        new choreo.trajectory.SwerveSample(0, 1, 1, 0, 1, 0, 0, 0, 0, 0, new double[4], new double[4]),
                        new choreo.trajectory.SwerveSample(1, 2, 1, 0, 1, 0, 0, 0, 0, 0, new double[4], new double[4]),
                        new choreo.trajectory.SwerveSample(2, 3, 1, 0, 1, 0, 0, 0, 0, 0, new double[4], new double[4]));
        return new choreo.trajectory.Trajectory<>(
                name, samples, List.of(), List.of(new choreo.trajectory.EventMarker(1.0, event)));
    }

    @AfterEach
    void restore() {
        BehaviorTreeAuto.message_log_ = edu.wpi.first.wpilibj.DataLogManager::log;
        AutoTrajectories.loader_ = name -> choreo.Choreo.loadTrajectory(name).map(t -> t);
    }

    BehaviorTreeAuto auto(String body) {
        xml.set(doc(body));
        BehaviorTreeAuto auto =
                new BehaviorTreeAuto(factory, "Demo", xml::get, null, new ParameterStore.InMemory());
        auto.reload(); // constructing reads nothing; AutoManager's boot check is the first load
        return auto;
    }

    @Test
    void runsFromTheLoopWithoutTheCommandScheduler() {
        BehaviorTreeAuto auto = auto("<Sequence><Step id=\"a\"/><Step id=\"b\"/></Sequence>");
        assertFalse(auto.isRunning());
        auto.start();
        assertTrue(auto.isRunning());
        auto.periodic(); // a starts
        auto.periodic(); // a done, b starts
        assertTrue(auto.isRunning());
        auto.periodic(); // b done: the root succeeds and the run finishes on its own
        assertFalse(auto.isRunning());
        assertEquals(List.of("a", "b"), ran);
        auto.periodic(); // nothing more happens after it finished
        assertEquals(List.of("a", "b"), ran);
    }

    @Test
    void stopHaltsTheRunningNodeAndEachStartIsAFreshTree() {
        BehaviorTreeAuto auto = auto("<Step id=\"a\"/>");
        auto.start();
        auto.periodic();
        auto.stop(true);
        assertFalse(auto.isRunning());
        assertEquals(List.of("a", "halted a"), ran);
        auto.start();
        auto.periodic();
        assertEquals(List.of("a", "halted a", "a"), ran, "a fresh tree starts at the first node");
    }

    @Test
    void aBackgroundPrepareChangesNothingUntilItsStepRuns() {
        BehaviorTreeAuto auto = auto("<Step id=\"old\"/>");
        xml.set(doc("<Step id=\"new\"/>"));
        Runnable use = auto.prepare(false); // what AutoManager runs on its loader thread
        auto.start();
        auto.periodic();
        assertEquals(List.of("old"), ran, "still the old tree until the returned step runs");
        auto.stop(true);
        use.run(); // robot thread
        auto.start();
        auto.periodic();
        assertEquals("new", ran.get(ran.size() - 1));
    }

    @Test
    void aMissingPathIsALoadErrorAndUnloadsTheTree() {
        AutoTrajectories.loader_ = name -> Optional.empty();
        BehaviorTreeAuto auto = auto("<Step id=\"good\"/>");
        assertTrue(auto.isLoaded());
        xml.set(doc("<Follow trajectory=\"Nowhere\"/>"));
        auto.prepare(true).run();
        assertTrue(auto.getError().contains("Nowhere"), auto.getError());
        assertFalse(auto.isLoaded(), "no fallback to an earlier tree or earlier paths");
        assertEquals(List.of(), auto.getTrajectoryNames());
        auto.start();
        assertFalse(auto.isRunning());
        auto.periodic();
        assertEquals(List.of(), ran, "nothing runs");
    }

    @Test
    void aFixedFileLoadsAgain() {
        BehaviorTreeAuto auto = auto("<NoSuchNode/>");
        assertFalse(auto.isLoaded());
        xml.set(doc("<Step id=\"fixed\"/>"));
        auto.prepare(false).run();
        assertTrue(auto.isLoaded(), auto.getError());
        assertEquals("", auto.getError());
    }

    @Test
    void anEventNoPathHasIsALoadError() {
        AutoTrajectories.loader_ = name -> Optional.of(pathWithEvent(name, "Intake Out"));
        BehaviorTreeAuto auto =
                auto("<Sequence><Follow trajectory=\"P1\"/><WaitEvent event=\"Intake Out\"/></Sequence>");
        auto.prepare(false).run();
        assertEquals("", auto.getError());

        xml.set(doc("<Sequence><Follow trajectory=\"P1\"/><WaitEvent event=\"Intake Ou\"/></Sequence>"));
        auto.prepare(false).run();
        assertTrue(auto.getError().contains("[Intake Ou]"), auto.getError());
        assertFalse(auto.isLoaded());
        assertTrue(auto.getError().contains("[Intake Out]"), "lists the markers there are");
    }

    @Test
    void aBrokenAutoNeverRuns() {
        BehaviorTreeAuto auto = auto("<NoSuchNode/>");
        assertFalse(auto.isLoaded());
        auto.start();
        assertFalse(auto.isRunning(), "nothing to run");
        auto.periodic();
        assertEquals(List.of(), ran);
    }

    @Test
    void commandAutosAreRoutinesToo() {
        AutoRoutine wrapped = AutoRoutine.of("Wait", Commands.none());
        assertEquals("Wait", wrapped.getName());
        assertFalse(wrapped.isRunning());
        AutoRoutine none = AutoRoutine.none("Do_Nothing");
        none.start();
        assertFalse(none.isRunning());
        // Auto subclasses are routines directly, with the same name as before
        class MyAuto extends Auto {}
        AutoRoutine mine = new MyAuto();
        assertEquals("MyAuto", mine.getName());
        assertFalse(mine.isRunning());
    }
}
