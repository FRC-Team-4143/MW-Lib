package com.marswars.bt.monitor;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.support.FakeClock;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class BehaviorTreeMonitorTest {
    static final String XML =
            "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\"><Sequence name=\"seq\">"
                    + "<Sleep msec=\"1000\"/><AlwaysSuccess/></Sequence></BehaviorTree></root>";

    final FakeClock clock = new FakeClock();
    NetworkTableInstance nt;

    @BeforeEach
    void setUp() {
        nt = NetworkTableInstance.create();
    }

    @AfterEach
    void tearDown() {
        nt.close();
    }

    /** Paths in XML: "seq" (named), "Sleep::2" and "AlwaysSuccess::3" (id plus uid). */
    @Test
    void publishesTransitionsOverNt() {
        BehaviorTree tree = new BehaviorTreeFactory(clock).createTreeFromText(XML);
        BehaviorTreeMonitor monitor =
                new BehaviorTreeMonitor("Demo", new BehaviorTreeMonitor.NtSink(nt));
        StringArraySubscriber transitions =
                nt.getStringArrayTopic("/BehaviorTree/Demo/Transitions").subscribe(new String[] {"unset"});

        monitor.publishStart(tree);
        monitor.publishTransitions();
        nt.flushLocal();
        assertArrayEquals(new String[0], transitions.get(), "nothing has run yet");

        tree.tickOnce();
        monitor.publishTransitions();
        nt.flushLocal();
        assertArrayEquals(new String[] {"seq: RUNNING", "Sleep::2: RUNNING"}, transitions.get());

        clock.advance(1.0);
        tree.tickOnce();
        monitor.publishTransitions();
        nt.flushLocal();
        List<String> second = List.of(transitions.get());
        assertTrue(second.contains("Sleep::2: SUCCESS"), second.toString());
        assertTrue(second.contains("AlwaysSuccess::3: SUCCESS"), second.toString());
        assertTrue(second.indexOf("Sleep::2: SUCCESS") < second.indexOf("seq: SUCCESS"),
                "a child's change is listed before its parent's: " + second);
    }

    @Test
    void anEmptyListIsWrittenOnceSoARepeatedTransitionIsStillTwoEntries() {
        BehaviorTree tree = new BehaviorTreeFactory(clock).createTreeFromText(XML);
        List<String> writes = new ArrayList<>();
        BehaviorTreeMonitor.Sink sink =
                new BehaviorTreeMonitor.Sink() {
                    @Override
                    public void putString(String key, String value) {
                        writes.add(key + "=" + value);
                    }

                    @Override
                    public void putStringArray(String key, String[] values) {
                        writes.add(key + "=" + List.of(values));
                    }
                };
        BehaviorTreeMonitor monitor = new BehaviorTreeMonitor("Demo", sink);
        monitor.publishStart(tree);
        monitor.publishTransitions(); // empty, written
        monitor.publishTransitions(); // still empty, not written again
        tree.tickOnce();
        monitor.publishTransitions();
        monitor.publishTransitions(); // empty again after the tick's entries, written once
        monitor.publishTransitions();
        assertEquals(
                List.of(
                        "BehaviorTree/Demo/Transitions=[]",
                        "BehaviorTree/Demo/Transitions=[seq: RUNNING, Sleep::2: RUNNING]",
                        "BehaviorTree/Demo/Transitions=[]"),
                writes);
    }

    @Test
    void detachStopsRecording() {
        BehaviorTree tree = new BehaviorTreeFactory(clock).createTreeFromText(XML);
        List<String> writes = new ArrayList<>();
        BehaviorTreeMonitor monitor =
                new BehaviorTreeMonitor("Demo", (k, v) -> writes.add(k + "=" + v));
        monitor.publishStart(tree);
        monitor.publishTransitions();
        writes.clear();
        monitor.detach();
        tree.tickOnce();
        monitor.publishTransitions();
        assertEquals(List.of(), writes, "no listener, nothing pending, empty already written");
    }
}
