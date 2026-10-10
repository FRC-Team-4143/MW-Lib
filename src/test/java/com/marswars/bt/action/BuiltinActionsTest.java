package com.marswars.bt.action;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.marswars.bt.support.BtTestBase;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class BuiltinActionsTest extends BtTestBase {

    @Test
    void sleepAndWaitUseTreeClock() {
        SleepNode sleep = new SleepNode("s", cfg(Map.of("msec", "300")));
        assertEquals(R, sleep.executeTick());
        clock.advance(0.29);
        assertEquals(R, sleep.executeTick());
        clock.advance(0.02);
        assertEquals(S, sleep.executeTick());

        WaitNode wait = new WaitNode("w", cfg(Map.of("seconds", "0")));
        assertEquals(S, wait.executeTick(), "zero wait succeeds immediately");
    }

    @Test
    void tunableWaitReadsRegistryAtStart() {
        TunableRegistry.InMemory registry = new TunableRegistry.InMemory();
        TunableWaitNode tw =
                new TunableWaitNode(
                        "tw", cfg(Map.of("key", "Auto/Wait", "default_seconds", "1")), registry);
        registry.set("Auto/Wait", 2.0);
        assertEquals(R, tw.executeTick());
        clock.advance(1.5);
        assertEquals(R, tw.executeTick());
        clock.advance(0.6);
        assertEquals(S, tw.executeTick());
    }

    @Test
    void setBlackboardLiteralAndCopy() {
        new SetBlackboardNode("a", cfg(Map.of("value", "42", "output_key", "answer")))
                .executeTick();
        assertEquals("42", bb.get("answer"));
        assertEquals(42, bb.get("answer", Integer.class));
        new SetBlackboardNode("b", cfg(Map.of("value", "{answer}", "output_key", "{copy}")))
                .executeTick();
        assertEquals("42", bb.get("copy"));
        new UnsetBlackboardNode("c", cfg(Map.of("key", "copy"))).executeTick();
        assertEquals(null, bb.get("copy"));
    }

    @Test
    void runCommandDrivesLifecycle() {
        List<String> log = new ArrayList<>();
        int[] executes = {0};
        RunCommandNode node =
                new RunCommandNode(
                        "cmd",
                        cfg(),
                        n ->
                                new Command() {
                                    @Override
                                    public void initialize() {
                                        log.add("init");
                                    }

                                    @Override
                                    public void execute() {
                                        executes[0]++;
                                    }

                                    @Override
                                    public boolean isFinished() {
                                        return executes[0] >= 2;
                                    }

                                    @Override
                                    public void end(boolean interrupted) {
                                        log.add("end:" + interrupted);
                                    }
                                });
        assertEquals(R, node.executeTick());
        assertEquals(S, node.executeTick());
        assertEquals(List.of("init", "end:false"), log);

        executes[0] = -100;
        node.resetStatus();
        node.executeTick();
        node.haltNode();
        assertEquals(List.of("init", "end:false", "init", "end:true"), log);
    }
}
