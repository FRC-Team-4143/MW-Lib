package com.marswars.bt.monitor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.support.FakeClock;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import javax.xml.parsers.DocumentBuilderFactory;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

class BehaviorTreeFileLoggerTest {
    static final String XML =
            "<root BTCPP_format=\"4\" main_tree_to_execute=\"Demo\">"
                    + "<BehaviorTree ID=\"Demo\"><Sequence name=\"root\">"
                    + "<AlwaysSuccess/>"
                    + "<Sleep msec=\"{wait_msec}\"/>"
                    + "</Sequence></BehaviorTree>"
                    + "<TreeNodesModel><SubTree ID=\"Demo\">"
                    + "<input_port name=\"wait_msec\" type=\"int\" default=\"500\"/>"
                    + "</SubTree></TreeNodesModel></root>";

    final FakeClock clock = new FakeClock();

    @AfterEach
    void tearDown() {
        BehaviorTreeFileLogger.disable();
    }

    @Test
    void writesEveryTransitionOfARun(@TempDir Path dir) throws Exception {
        BehaviorTreeFileLogger.enable(dir);
        clock.set(10.0);
        BehaviorTreeFactory factory = new BehaviorTreeFactory(clock);
        BehaviorTreeCommand cmd =
                new BehaviorTreeCommand(
                        "Demo",
                        () -> factory.createTreeFromText(XML),
                        new BehaviorTreeMonitor("Demo", (k, v) -> {}));
        cmd.initialize();
        cmd.execute();
        clock.advance(0.6);
        cmd.execute();
        cmd.end(false);

        Path file =
                BehaviorTreeFileLogger.active().orElseThrow().lastWrite().get(5, TimeUnit.SECONDS);
        assertTrue(file.getFileName().toString().startsWith("Demo_"));
        assertTrue(file.getFileName().toString().endsWith(".btlog.xml"));

        Document doc =
                DocumentBuilderFactory.newInstance().newDocumentBuilder().parse(file.toFile());
        Element root = doc.getDocumentElement();
        assertEquals("BehaviorTreeLog", root.getTagName());
        assertEquals("mwlib-btlog", root.getAttribute("format"));
        assertEquals("Demo", root.getAttribute("tree_id"));
        assertEquals("SUCCESS", root.getAttribute("result"));
        assertEquals("10.0000", root.getAttribute("t_start"));
        assertEquals("10.6000", root.getAttribute("t_end"));

        Element param = (Element) doc.getElementsByTagName("Parameter").item(0);
        assertEquals("wait_msec", param.getAttribute("name"));
        assertEquals("int", param.getAttribute("type"));
        assertEquals("500", param.getAttribute("value"));

        NodeList nodes = doc.getElementsByTagName("Node");
        assertEquals(3, nodes.getLength());
        assertEquals("root", ((Element) nodes.item(0)).getAttribute("name"));
        assertEquals("1", ((Element) nodes.item(1)).getAttribute("parent"));

        List<String> transitions = new ArrayList<>();
        NodeList ts = doc.getElementsByTagName("T");
        for (int i = 0; i < ts.getLength(); i++) {
            Element t = (Element) ts.item(i);
            transitions.add(
                    t.getAttribute("t") + " " + t.getAttribute("uid") + " "
                            + t.getAttribute("prev") + ">" + t.getAttribute("status"));
        }
        assertEquals(
                List.of(
                        "10.0000 1 IDLE>RUNNING",
                        "10.0000 2 IDLE>SUCCESS", // never visible in the end-of-tick Status string
                        "10.0000 3 IDLE>RUNNING",
                        "10.6000 3 RUNNING>SUCCESS",
                        "10.6000 2 SUCCESS>IDLE",
                        "10.6000 3 SUCCESS>IDLE",
                        "10.6000 1 RUNNING>SUCCESS",
                        "10.6000 1 SUCCESS>IDLE"),
                transitions);

        String treeXml = doc.getElementsByTagName("TreeXml").item(0).getTextContent();
        assertEquals(XML, treeXml);
    }

    @Test
    void interruptedRunAndNoLoggerMeansNoFile(@TempDir Path dir) throws Exception {
        BehaviorTreeFactory factory = new BehaviorTreeFactory(clock);
        BehaviorTreeCommand cmd =
                new BehaviorTreeCommand(
                        "Demo",
                        () -> factory.createTreeFromText(XML),
                        new BehaviorTreeMonitor("Demo", (k, v) -> {}));
        cmd.initialize(); // logger disabled: nothing recorded
        cmd.execute();
        cmd.end(true);
        assertEquals(0, java.nio.file.Files.list(dir).count());

        BehaviorTreeFileLogger.enable(dir);
        cmd.initialize();
        cmd.execute();
        cmd.end(true);
        Path file =
                BehaviorTreeFileLogger.active().orElseThrow().lastWrite().get(5, TimeUnit.SECONDS);
        String text = java.nio.file.Files.readString(file);
        assertTrue(text.contains("result=\"INTERRUPTED\""));
        assertTrue(text.contains("prev=\"RUNNING\" status=\"IDLE\""), "halt transitions recorded");
    }
}
