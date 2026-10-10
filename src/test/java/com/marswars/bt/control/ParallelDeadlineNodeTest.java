package com.marswars.bt.control;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.support.BtTestBase;
import com.marswars.bt.support.ScriptedAction;
import org.junit.jupiter.api.Test;

class ParallelDeadlineNodeTest extends BtTestBase {

    @Test
    void endsWhenDeadlineFinishesAndHaltsTheRest() {
        ScriptedAction path = action("path", R, R, S);
        ScriptedAction events = action("events", R);
        ParallelDeadlineNode pd = with(new ParallelDeadlineNode("pd", cfg()), path, events);

        assertEquals(R, pd.executeTick());
        assertEquals(R, pd.executeTick());
        assertEquals(S, pd.executeTick());
        assertEquals(3, events.ticks(), "others run alongside every tick");
        assertEquals(1, events.halts, "still-running sibling halted at the deadline");
        assertEquals(I, events.getStatus());
        assertEquals(I, path.getStatus());
    }

    @Test
    void othersFinishingEarlyDoNotEndItAndAreNotRestarted() {
        ScriptedAction path = action("path", R, R, R, S);
        ScriptedAction quick = action("quick", S);
        ParallelDeadlineNode pd = with(new ParallelDeadlineNode("pd", cfg()), path, quick);

        assertEquals(R, pd.executeTick());
        assertEquals(R, pd.executeTick());
        assertEquals(R, pd.executeTick());
        assertEquals(S, pd.executeTick());
        assertEquals(1, quick.starts, "a finished child is not ticked again");
    }

    @Test
    void returnsDeadlineFailureAndIgnoresOthersFailure() {
        ParallelDeadlineNode fails =
                with(new ParallelDeadlineNode("a", cfg()), action("d", R, F), action("o", R));
        assertEquals(R, fails.executeTick());
        assertEquals(F, fails.executeTick());

        ParallelDeadlineNode ignores =
                with(new ParallelDeadlineNode("b", cfg()), action("d", R, S), action("o", F));
        assertEquals(R, ignores.executeTick());
        assertEquals(S, ignores.executeTick());
    }

    @Test
    void haltResetsAndRunsAgainFromScratch() {
        ScriptedAction path = action("path", R);
        ScriptedAction quick = action("quick", S);
        ParallelDeadlineNode pd = with(new ParallelDeadlineNode("pd", cfg()), path, quick);
        pd.executeTick();
        pd.haltNode();
        assertEquals(1, path.halts);
        pd.executeTick();
        assertEquals(2, quick.starts, "completed set cleared by halt");
    }

    @Test
    void availableFromXmlWithoutPark() {
        BehaviorTree tree =
                new BehaviorTreeFactory(clock)
                        .createTreeFromText(
                                "<root BTCPP_format=\"4\"><BehaviorTree ID=\"M\">"
                                        + "<ParallelDeadline><Sleep msec=\"500\"/>"
                                        + "<Sequence><AlwaysSuccess/><AlwaysSuccess/></Sequence>"
                                        + "</ParallelDeadline></BehaviorTree></root>");
        assertEquals(R, tree.tickOnce(), "branch done, deadline still sleeping");
        clock.advance(0.6);
        assertEquals(S, tree.tickOnce());
    }
}
