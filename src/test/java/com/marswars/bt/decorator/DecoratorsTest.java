package com.marswars.bt.decorator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.marswars.bt.action.AlwaysSuccessNode;
import com.marswars.bt.support.BtTestBase;
import com.marswars.bt.support.ScriptedAction;
import java.util.Map;
import org.junit.jupiter.api.Test;

class DecoratorsTest extends BtTestBase {

    @Test
    void inverter() {
        assertEquals(F, with(new InverterNode("i", cfg()), action("a", S)).executeTick());
        assertEquals(S, with(new InverterNode("i", cfg()), action("a", F)).executeTick());
        assertEquals(R, with(new InverterNode("i", cfg()), action("a", R)).executeTick());
    }

    @Test
    void forceSuccessAndFailure() {
        assertEquals(S, with(new ForceSuccessNode("f", cfg()), action("a", F)).executeTick());
        assertEquals(F, with(new ForceFailureNode("f", cfg()), action("a", S)).executeTick());
        assertEquals(R, with(new ForceFailureNode("f", cfg()), action("a", R)).executeTick());
    }

    @Test
    void repeatFiniteRunsSyncChildWithinOneTick() {
        ScriptedAction a = action("a", S);
        RepeatNode rep = with(new RepeatNode("rep", cfg(Map.of("num_cycles", "3"))), a);
        assertEquals(S, rep.executeTick());
        assertEquals(3, a.starts);
    }

    @Test
    void repeatAsyncChildAndFailure() {
        ScriptedAction a = action("a", R, S, R, F);
        RepeatNode rep = with(new RepeatNode("rep", cfg(Map.of("num_cycles", "5"))), a);
        assertEquals(R, rep.executeTick());
        assertEquals(R, rep.executeTick()); // first cycle done, second started -> RUNNING
        assertEquals(F, rep.executeTick());
    }

    @Test
    void repeatForeverYieldsEachCycle() {
        ScriptedAction a = action("a", S);
        RepeatNode rep = with(new RepeatNode("rep", cfg(Map.of("num_cycles", "-1"))), a);
        for (int i = 0; i < 5; i++) {
            assertEquals(R, rep.executeTick());
        }
        assertEquals(5, a.starts);
    }

    @Test
    void retryUntilSuccessful() {
        ScriptedAction a = action("a", F, F, S);
        RetryNode retry = with(new RetryNode("retry", cfg(Map.of("num_attempts", "3"))), a);
        assertEquals(S, retry.executeTick());
        assertEquals(3, a.starts);

        ScriptedAction b = action("b", F);
        RetryNode exhausted = with(new RetryNode("retry", cfg(Map.of("num_attempts", "2"))), b);
        assertEquals(F, exhausted.executeTick());
        assertEquals(2, b.starts);
        assertEquals("RetryUntilSuccessful", exhausted.getRegistrationId());
    }

    @Test
    void keepRunningUntilFailureParksForever() {
        KeepRunningUntilFailureNode park =
                with(new KeepRunningUntilFailureNode("park", cfg()), new AlwaysSuccessNode("s", cfg()));
        for (int i = 0; i < 3; i++) {
            assertEquals(R, park.executeTick());
        }
        assertEquals(
                F,
                with(new KeepRunningUntilFailureNode("k", cfg()), action("a", F)).executeTick());
    }

    @Test
    void timeoutHaltsSlowChild() {
        ScriptedAction slow = action("slow", R);
        TimeoutNode to = with(new TimeoutNode("to", cfg(Map.of("msec", "500"))), slow);
        assertEquals(R, to.executeTick());
        clock.advance(0.4);
        assertEquals(R, to.executeTick());
        clock.advance(0.2);
        assertEquals(F, to.executeTick());
        assertEquals(1, slow.halts);
        assertEquals(I, slow.getStatus());
    }

    @Test
    void timeoutPassesFastResult() {
        ScriptedAction fast = action("fast", R, S);
        TimeoutNode to = with(new TimeoutNode("to", cfg(Map.of("msec", "500"))), fast);
        assertEquals(R, to.executeTick());
        clock.advance(0.1);
        assertEquals(S, to.executeTick());
        // re-armed: a fresh run gets a fresh 500 ms
        fast.script(R);
        clock.advance(10);
        assertEquals(R, to.executeTick());
    }

    @Test
    void delayWaitsThenTicksChild() {
        ScriptedAction a = action("a", S);
        DelayNode delay = with(new DelayNode("d", cfg(Map.of("delay_msec", "250"))), a);
        assertEquals(R, delay.executeTick());
        clock.advance(0.2);
        assertEquals(R, delay.executeTick());
        assertEquals(0, a.starts);
        clock.advance(0.1);
        assertEquals(S, delay.executeTick());
        assertEquals(1, a.starts);
        assertEquals(R, delay.executeTick(), "re-armed after completion");
    }

    @Test
    void runOnceSkipsAfterFirstCompletion() {
        ScriptedAction a = action("a", R, F);
        RunOnceNode once = with(new RunOnceNode("once", cfg()), a);
        assertEquals(R, once.executeTick());
        assertEquals(F, once.executeTick());
        assertEquals(K, once.executeTick());
        assertEquals(I, once.getStatus());

        RunOnceNode keep =
                with(new RunOnceNode("keep", cfg(Map.of("then_skip", "false"))), action("b", S));
        assertEquals(S, keep.executeTick());
        assertEquals(S, keep.executeTick());
    }
}
