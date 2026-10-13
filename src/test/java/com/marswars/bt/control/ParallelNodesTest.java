package com.marswars.bt.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.marswars.bt.core.BtException;
import com.marswars.bt.support.BtTestBase;
import com.marswars.bt.support.ScriptedAction;
import java.util.Map;
import org.junit.jupiter.api.Test;

class ParallelNodesTest extends BtTestBase {

    @Test
    void defaultNeedsAllToSucceedAndDoesNotReTickCompleted() {
        ScriptedAction a = action("a", S);
        ScriptedAction b = action("b", R, R, S);
        ParallelNode par = with(new ParallelNode("par", cfg()), a, b);

        assertEquals(R, par.executeTick());
        assertEquals(R, par.executeTick());
        assertEquals(S, par.executeTick());
        assertEquals(1, a.starts, "completed child not re-ticked");
        assertEquals(I, a.getStatus());
    }

    @Test
    void successCountOneHaltsSiblingsOnFirstSuccess() {
        ScriptedAction path = action("path", R, R, S);
        ScriptedAction events = action("events", R);
        ParallelNode par =
                with(
                        new ParallelNode(
                                "par", cfg(Map.of("success_count", "1", "failure_count", "1"))),
                        path,
                        events);

        assertEquals(R, par.executeTick());
        assertEquals(R, par.executeTick());
        assertEquals(S, par.executeTick());
        assertEquals(1, events.halts, "still-running sibling halted on early success");
        assertEquals(I, events.getStatus());
    }

    @Test
    void failureCountReached() {
        ScriptedAction a = action("a", F);
        ScriptedAction b = action("b", R);
        ParallelNode par = with(new ParallelNode("par", cfg()), a, b);
        assertEquals(F, par.executeTick());
        assertEquals(0, b.starts, "default success=-1 becomes impossible after first failure");
    }

    @Test
    void failsWhenSuccessBecomesImpossible() {
        ScriptedAction a = action("a", F);
        ScriptedAction b = action("b", R);
        ScriptedAction c = action("c", R);
        ParallelNode par =
                with(
                        new ParallelNode(
                                "par", cfg(Map.of("success_count", "3", "failure_count", "3"))),
                        a,
                        b,
                        c);
        assertEquals(F, par.executeTick());
    }

    @Test
    void unreachableThresholdThrows() {
        ParallelNode par =
                with(new ParallelNode("par", cfg(Map.of("success_count", "3"))), action("a", R));
        assertThrows(BtException.class, par::executeTick);
    }

    @Test
    void parallelAllWaitsForEveryChild() {
        ScriptedAction a = action("a", F);
        ScriptedAction b = action("b", R, S);
        ParallelAllNode par = with(new ParallelAllNode("all", cfg()), a, b);
        assertEquals(R, par.executeTick());
        assertEquals(F, par.executeTick(), "one failure >= max_failures=1");
        assertEquals(2, b.ticks());
    }

    @Test
    void parallelAllMaxFailures() {
        ParallelAllNode par =
                with(
                        new ParallelAllNode("all", cfg(Map.of("max_failures", "2"))),
                        action("a", F),
                        action("b", S));
        assertEquals(S, par.executeTick());
    }

    @Test
    void ifThenElseBranches() {
        var cond = flag("cond", true);
        ScriptedAction then = action("then", R, S);
        ScriptedAction otherwise = action("else", S);
        IfThenElseNode ite = with(new IfThenElseNode("ite", cfg()), cond, then, otherwise);

        assertEquals(R, ite.executeTick());
        cond.value = false; // not re-checked while the branch runs
        assertEquals(S, ite.executeTick());
        assertEquals(1, cond.ticks);
        assertEquals(S, ite.executeTick());
        assertEquals(1, otherwise.starts);
    }

    @Test
    void ifThenElseTwoChildrenFailureAndArity() {
        IfThenElseNode ite = with(new IfThenElseNode("ite", cfg()), flag("c", false), action("t"));
        assertEquals(F, ite.executeTick());
        IfThenElseNode bad = with(new IfThenElseNode("bad", cfg()), flag("c", true));
        assertThrows(BtException.class, bad::executeTick);
    }

    @Test
    void whileDoElseHaltsBranchWhenConditionFlips() {
        var cond = flag("cond", true);
        ScriptedAction doing = action("do", R);
        ScriptedAction otherwise = action("else", R);
        WhileDoElseNode wde = with(new WhileDoElseNode("wde", cfg()), cond, doing, otherwise);

        assertEquals(R, wde.executeTick());
        cond.value = false;
        assertEquals(R, wde.executeTick());
        assertEquals(1, doing.halts);
        assertEquals(1, otherwise.starts);
    }
}
