package com.marswars.bt.monitor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.support.FakeClock;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
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

    @Test
    void publishesStructureXmlAndStatusOverNt() {
        BehaviorTree tree = new BehaviorTreeFactory(clock).createTreeFromText(XML);
        BehaviorTreeMonitor monitor =
                new BehaviorTreeMonitor("Demo", new BehaviorTreeMonitor.NtSink(nt));
        StringSubscriber status = nt.getStringTopic("/BehaviorTree/Demo/Status").subscribe("");
        StringSubscriber structure =
                nt.getStringTopic("/BehaviorTree/Demo/Structure").subscribe("");
        StringSubscriber xml = nt.getStringTopic("/BehaviorTree/Demo/Xml").subscribe("");

        monitor.publishTree(tree);
        nt.flushLocal();
        assertEquals("III", status.get());
        assertEquals(XML, xml.get());
        JsonObject s = JsonParser.parseString(structure.get()).getAsJsonObject();
        assertEquals("Main", s.get("tree_id").getAsString());
        JsonObject first = s.getAsJsonArray("nodes").get(0).getAsJsonObject();
        assertEquals("Sequence", first.get("type").getAsString());
        assertEquals("seq", first.get("name").getAsString());
        assertTrue(first.get("parent").isJsonNull());
        assertEquals(1, s.getAsJsonArray("nodes").get(1).getAsJsonObject().get("parent").getAsInt());

        tree.tickOnce();
        monitor.publishStatus(tree);
        nt.flushLocal();
        assertEquals("RRI", status.get());
    }

    @Test
    void statusPublishedOnlyOnChange() {
        BehaviorTree tree = new BehaviorTreeFactory(clock).createTreeFromText(XML);
        List<String> writes = new ArrayList<>();
        BehaviorTreeMonitor monitor =
                new BehaviorTreeMonitor("Demo", (k, v) -> writes.add(k + "=" + v));
        monitor.publishTree(tree);
        writes.clear();
        tree.tickOnce();
        monitor.publishStatus(tree);
        tree.tickOnce();
        monitor.publishStatus(tree);
        assertEquals(List.of("BehaviorTree/Demo/Status=RRI"), writes);
    }
}
