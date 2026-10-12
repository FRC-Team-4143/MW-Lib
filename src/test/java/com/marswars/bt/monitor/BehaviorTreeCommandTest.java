package com.marswars.bt.monitor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.BtException;
import com.marswars.bt.core.NodeKind;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.support.FakeClock;
import com.marswars.bt.support.ScriptedAction;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

class BehaviorTreeCommandTest {
    final FakeClock clock = new FakeClock();
    final List<String> writes = new ArrayList<>();
    final List<String> messages = new ArrayList<>();
    final List<ScriptedAction> actions = new ArrayList<>();
    BehaviorTreeFactory factory;
    Consumer<String> saved_log;

    @BeforeEach
    void setUp() {
        saved_log = BehaviorTreeRunner.message_log_;
        BehaviorTreeRunner.message_log_ = messages::add;
        factory = new BehaviorTreeFactory(clock);
        factory.registerNodeType(
                "Drive",
                NodeKind.ACTION,
                "",
                List.of(),
                (name, cfg) -> {
                    ScriptedAction a = new ScriptedAction(name, cfg, NodeStatus.RUNNING);
                    actions.add(a);
                    return a;
                });
    }

    @AfterEach
    void tearDown() {
        BehaviorTreeRunner.message_log_ = saved_log;
    }

    BehaviorTreeCommand command(String body) {
        String xml =
                "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">"
                        + body
                        + "</BehaviorTree></root>";
        return new BehaviorTreeCommand(
                "Test",
                () -> factory.createTreeFromText(xml),
                new BehaviorTreeMonitor("Test", (k, v) -> writes.add(k + "=" + v)));
    }

    @Test
    void runsToCompletionAndRebuildsEachRun() {
        BehaviorTreeCommand cmd =
                command("<Sequence><Sleep msec=\"500\"/><AlwaysSuccess/></Sequence>");
        cmd.initialize();
        BehaviorTree first = cmd.getTree();
        cmd.execute();
        assertFalse(cmd.isFinished());
        clock.advance(0.6);
        cmd.execute();
        assertTrue(cmd.isFinished());
        cmd.end(false);
        assertTrue(writes.contains("BehaviorTree/Test/Result=SUCCESS"));
        assertTrue(writes.contains("BehaviorTree/Active=Test"));
        assertEquals("BehaviorTree/Active=", writes.get(writes.size() - 1));

        cmd.initialize();
        assertNotSame(first, cmd.getTree(), "fresh tree per run");
    }

    @Test
    void interruptHaltsRunningLeaves() {
        BehaviorTreeCommand cmd = command("<Sequence><Drive/></Sequence>");
        cmd.initialize();
        cmd.execute();
        cmd.execute();
        assertFalse(cmd.isFinished());
        cmd.end(true);
        assertEquals(1, actions.get(actions.size() - 1).halts);
        assertTrue(writes.contains("BehaviorTree/Test/Result=INTERRUPTED"));
        assertEquals("III".substring(0, 2), cmd.getTree().statusString());
    }

    @Test
    void supplierFailureFinishesImmediately() {
        BehaviorTreeCommand cmd =
                new BehaviorTreeCommand(
                        "Bad",
                        () -> {
                            throw new BtException("broken xml");
                        },
                        new BehaviorTreeMonitor("Bad", (k, v) -> writes.add(k + "=" + v)));
        cmd.initialize();
        cmd.execute();
        assertTrue(cmd.isFinished());
        cmd.end(false);
        assertTrue(writes.contains("BehaviorTree/Bad/Error=cannot build tree: broken xml"));
        assertTrue(writes.contains("BehaviorTree/Bad/Result=ERROR"));
        assertEquals(1, messages.size());
    }

    @Test
    void aNodeThatThrowsWhenHaltedStillEndsTheRun() {
        factory.registerNodeType(
                "BadHalt",
                NodeKind.ACTION,
                "",
                List.of(),
                (name, cfg) ->
                        new com.marswars.bt.core.StatefulActionNode(name, cfg) {
                            @Override
                            protected NodeStatus onStart() {
                                return NodeStatus.RUNNING;
                            }

                            @Override
                            protected NodeStatus onRunning() {
                                return NodeStatus.RUNNING;
                            }

                            @Override
                            protected void onHalted() {
                                throw new IllegalStateException("motor gone");
                            }
                        });
        BehaviorTreeCommand cmd = command("<BadHalt/>");
        cmd.initialize();
        cmd.execute();
        cmd.end(true); // must not throw out of disabledInit / teleopInit
        assertTrue(writes.contains("BehaviorTree/Test/Result=INTERRUPTED"), writes.toString());
        assertTrue(
                writes.stream()
                        .anyMatch(
                                w ->
                                        w.contains("exception while halting")
                                                && w.contains("motor gone")),
                writes.toString());
    }

    @Test
    void tickExceptionFailsTheRun() {
        factory.registerSimpleAction(
                "Boom",
                "",
                List.of(),
                n -> {
                    throw new IllegalStateException("kaboom");
                });
        BehaviorTreeCommand cmd = command("<Boom/>");
        cmd.initialize();
        cmd.execute();
        assertTrue(cmd.isFinished());
        assertEquals(NodeStatus.FAILURE, cmd.getLastStatus());
        assertTrue(messages.get(0).contains("kaboom"));
    }
}
