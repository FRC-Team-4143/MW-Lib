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
    }

    @AfterEach
    void restore() {
        BehaviorTreeAuto.message_log_ = edu.wpi.first.wpilibj.DataLogManager::log;
        AutoTrajectories.loader_ = name -> choreo.Choreo.loadTrajectory(name).map(t -> t);
    }

    BehaviorTreeAuto auto(String body) {
        xml.set(doc(body));
        return new BehaviorTreeAuto(factory, "Demo", xml::get, null, new ParameterStore.InMemory());
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
    void aBackgroundPrepareChangesNothingUntilTheRobotThreadFinishesIt() {
        BehaviorTreeAuto auto = auto("<Step id=\"old\"/>");
        xml.set(doc("<Step id=\"new\"/>"));
        auto.prepare(false); // what AutoManager runs on its loader thread
        auto.start();
        auto.periodic();
        assertEquals(List.of("old"), ran, "still the old tree before finishPrepare");
        auto.stop(true);
        auto.finishPrepare(); // robot thread
        auto.start();
        auto.periodic();
        assertEquals("new", ran.get(ran.size() - 1));
    }

    @Test
    void aMissingPathIsALoadErrorAndKeepsTheLastGoodTree() {
        AutoTrajectories.loader_ = name -> Optional.empty();
        BehaviorTreeAuto auto = auto("<Step id=\"good\"/>");
        assertTrue(auto.isLoaded());
        xml.set(doc("<Follow trajectory=\"Nowhere\"/>"));
        auto.prepare(true);
        auto.finishPrepare();
        assertTrue(auto.getError().contains("Nowhere"), auto.getError());
        auto.start();
        auto.periodic();
        assertEquals(List.of("good"), ran, "the last good tree still runs");
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
