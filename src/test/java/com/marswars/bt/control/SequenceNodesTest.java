package com.marswars.bt.control;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.marswars.bt.support.BtTestBase;
import com.marswars.bt.support.Flag;
import com.marswars.bt.support.ScriptedAction;
import org.junit.jupiter.api.Test;

class SequenceNodesTest extends BtTestBase {

    @Test
    void sequenceResumesAtRunningChildAndSucceeds() {
        ScriptedAction a = action("a", S);
        ScriptedAction b = action("b", R, R, S);
        SequenceNode seq = with(new SequenceNode("seq", cfg()), a, b);

        assertEquals(R, seq.executeTick());
        assertEquals(R, seq.executeTick());
        assertEquals(S, seq.executeTick());
        assertEquals(1, a.starts, "a is not re-ticked while b runs");
        assertEquals(3, b.ticks());
        assertEquals(I, a.getStatus(), "children reset on completion");
        assertEquals(I, b.getStatus());
    }

    @Test
    void sequenceFailureRestartsFromFirstChild() {
        ScriptedAction a = action("a", S);
        ScriptedAction b = action("b", F);
        SequenceNode seq = with(new SequenceNode("seq", cfg()), a, b);

        assertEquals(F, seq.executeTick());
        assertEquals(F, seq.executeTick());
        assertEquals(2, a.starts, "after FAILURE the sequence restarts at child 0");
    }

    @Test
    void sequenceHaltHaltsRunningChildAndRewinds() {
        ScriptedAction a = action("a", S);
        ScriptedAction b = action("b", R);
        SequenceNode seq = with(new SequenceNode("seq", cfg()), a, b);
        seq.executeTick();
        seq.haltNode();
        assertEquals(1, b.halts);
        assertEquals(I, seq.getStatus());
        seq.executeTick();
        assertEquals(2, a.starts);
    }

    @Test
    void sequenceAllSkippedIsSkipped() {
        SequenceNode seq = with(new SequenceNode("seq", cfg()), new Skip(cfg()), new Skip(cfg()));
        assertEquals(K, seq.executeTick());
        assertEquals(I, seq.getStatus());
    }

    @Test
    void sequenceWithMemoryResumesAtFailedChild() {
        ScriptedAction a = action("a", S);
        ScriptedAction b = action("b", F, S);
        SequenceWithMemoryNode seq = with(new SequenceWithMemoryNode("seq", cfg()), a, b);

        assertEquals(F, seq.executeTick());
        assertEquals(1, seq.currentChildIndex());
        assertEquals(S, seq.executeTick());
        assertEquals(1, a.starts, "already succeeded child not re-run");
        assertEquals(2, b.starts);
    }

    @Test
    void sequenceWithMemoryKeepsIndexAcrossHalt() {
        ScriptedAction a = action("a", S);
        ScriptedAction b = action("b", R);
        SequenceWithMemoryNode seq = with(new SequenceWithMemoryNode("seq", cfg()), a, b);
        seq.executeTick();
        seq.haltNode();
        assertEquals(1, b.halts);
        seq.executeTick();
        assertEquals(1, a.starts);
        assertEquals(2, b.starts);
    }

    @Test
    void fallbackTriesUntilSuccess() {
        ScriptedAction a = action("a", F);
        ScriptedAction b = action("b", R, S);
        ScriptedAction c = action("c", S);
        FallbackNode fb = with(new FallbackNode("fb", cfg()), a, b, c);

        assertEquals(R, fb.executeTick());
        assertEquals(S, fb.executeTick());
        assertEquals(1, a.starts);
        assertEquals(0, c.starts);
    }

    @Test
    void fallbackAllFail() {
        FallbackNode fb = with(new FallbackNode("fb", cfg()), action("a", F), action("b", F));
        assertEquals(F, fb.executeTick());
    }

    @Test
    void reactiveSequenceReticksConditionsAndHaltsOnFailure() {
        Flag guard = flag("guard", true);
        ScriptedAction act = action("act", R);
        ReactiveSequenceNode rs = with(new ReactiveSequenceNode("rs", cfg()), guard, act);

        assertEquals(R, rs.executeTick());
        assertEquals(R, rs.executeTick());
        assertEquals(2, guard.ticks, "condition re-ticked every tick");
        assertEquals(1, act.starts);
        assertEquals(1, act.runs);

        guard.value = false;
        assertEquals(F, rs.executeTick());
        assertEquals(1, act.halts, "running action halted when guard fails");
        assertEquals(I, act.getStatus());
    }

    @Test
    void reactiveSequenceHaltsOtherRunningChild() {
        ScriptedAction first = action("first", S);
        ScriptedAction second = action("second", R);
        ReactiveSequenceNode rs = with(new ReactiveSequenceNode("rs", cfg()), first, second);
        rs.executeTick();
        // first now becomes asynchronous: the running second must be halted
        first.script(R);
        assertEquals(R, rs.executeTick());
        assertEquals(1, second.halts);
    }

    @Test
    void reactiveSequenceThrowsOnSecondRunningWhenEnabled() {
        ReactiveSequenceNode.enableException(true);
        try {
            ScriptedAction first = action("first", S);
            ScriptedAction second = action("second", R);
            ReactiveSequenceNode rs = with(new ReactiveSequenceNode("rs", cfg()), first, second);
            rs.executeTick();
            first.script(R);
            org.junit.jupiter.api.Assertions.assertThrows(
                    com.marswars.bt.core.BtException.class, rs::executeTick);
        } finally {
            ReactiveSequenceNode.enableException(false);
        }
    }

    @Test
    void reactiveFallbackPreemptsWhenEarlierChildSucceeds() {
        Flag done = flag("done", false);
        ScriptedAction work = action("work", R);
        ReactiveFallbackNode rf = with(new ReactiveFallbackNode("rf", cfg()), done, work);

        assertEquals(R, rf.executeTick());
        assertEquals(R, rf.executeTick());
        done.value = true;
        assertEquals(S, rf.executeTick());
        assertEquals(1, work.halts);
    }

    /** Leaf that is always skipped. */
    static final class Skip extends com.marswars.bt.core.SyncActionNode {
        Skip(com.marswars.bt.core.NodeConfig c) {
            super("skip", c);
        }

        @Override
        protected com.marswars.bt.core.NodeStatus tick() {
            return com.marswars.bt.core.NodeStatus.SKIPPED;
        }
    }
}
