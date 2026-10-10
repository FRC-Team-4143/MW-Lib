package com.marswars.bt.monitor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.NodeSpec;
import com.marswars.bt.core.TreeSpec;
import com.marswars.bt.support.FakeClock;
import com.marswars.bt.xml.BtXmlParser;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class BehaviorTreeFileLoggerTest {
    static final String XML =
            "<root BTCPP_format=\"4\" main_tree_to_execute=\"Demo\">"
                    + "<BehaviorTree ID=\"Demo\"><Sequence name=\"root\">"
                    + "<AlwaysSuccess/>"
                    + "<SubTree ID=\"Wait\" ms=\"{wait_msec}\"/>"
                    + "</Sequence></BehaviorTree>"
                    + "<BehaviorTree ID=\"Wait\"><Sleep msec=\"{ms}\"/></BehaviorTree>"
                    + "<TreeNodesModel><SubTree ID=\"Demo\">"
                    + "<input_port name=\"wait_msec\" type=\"int\" default=\"500\"/>"
                    + "</SubTree></TreeNodesModel></root>";

    /** Decoded .btlog (same layout as BT.CPP FileLogger2). */
    record BtLog(int protocol, String xml, long startUsec, List<long[]> records) {
        static BtLog read(Path file) throws Exception {
            ByteBuffer b = ByteBuffer.wrap(Files.readAllBytes(file)).order(ByteOrder.LITTLE_ENDIAN);
            byte[] magic = new byte[18];
            b.get(magic);
            assertEquals("BTCPP4-FileLogger2", new String(magic, StandardCharsets.US_ASCII));
            int protocol = b.get();
            byte[] xml = new byte[b.getInt()];
            b.get(xml);
            long start = b.getLong();
            List<long[]> records = new ArrayList<>();
            assertEquals(0, b.remaining() % 9, "whole 9-byte records");
            while (b.remaining() >= 9) {
                long usec = 0;
                for (int i = 0; i < 6; i++) {
                    usec |= (b.get() & 0xFFL) << (8 * i);
                }
                long uid = b.getShort() & 0xFFFF;
                long status = b.get() & 0xFF;
                records.add(new long[] {usec, uid, status});
            }
            return new BtLog(protocol, new String(xml, StandardCharsets.UTF_8), start, records);
        }
    }

    final FakeClock clock = new FakeClock();

    @AfterEach
    void tearDown() {
        BehaviorTreeFileLogger.disable();
    }

    BehaviorTreeCommand command() {
        BehaviorTreeFactory factory = new BehaviorTreeFactory(clock);
        return new BehaviorTreeCommand(
                "Demo",
                () -> factory.createTreeFromText(XML),
                new BehaviorTreeMonitor("Demo", (k, v) -> {}));
    }

    @Test
    void writesBtcppFileLogger2(@TempDir Path dir) throws Exception {
        BehaviorTreeFileLogger.enable(dir);
        clock.set(10.0);
        long before = System.currentTimeMillis() * 1000;
        BehaviorTreeCommand cmd = command();
        cmd.initialize();
        cmd.execute();
        clock.advance(0.6);
        cmd.execute();
        cmd.end(false);

        Path file =
                BehaviorTreeFileLogger.active().orElseThrow().lastWrite().get(5, TimeUnit.SECONDS);
        assertTrue(file.getFileName().toString().matches("Demo_\\d{8}_\\d{6}_\\d{3}\\.btlog"));

        BtLog log = BtLog.read(file);
        assertEquals(1, log.protocol());
        assertTrue(log.startUsec() >= before - 1_000_000 && log.startUsec() <= before + 60_000_000L);

        // Embedded XML: BT.CPP WriteTreeToXML with metadata; parses as BT.CPP v4.
        assertTrue(log.xml().contains("<BehaviorTree ID=\"Demo\" _fullpath=\"\">"), log.xml());
        assertTrue(log.xml().contains("<Sequence name=\"root\" _uid=\"1\">"));
        assertTrue(log.xml().contains("<AlwaysSuccess name=\"AlwaysSuccess\" _uid=\"2\"/>"));
        assertTrue(log.xml().contains(
                "<SubTree ID=\"Wait\" _fullpath=\"Wait::3\" _uid=\"3\" ms=\"{wait_msec}\"/>"));
        assertTrue(log.xml().contains("<BehaviorTree ID=\"Wait\" _fullpath=\"Wait::3\">"));
        assertTrue(log.xml().contains("<Sleep name=\"Sleep\" _uid=\"4\" msec=\"{ms}\"/>"));
        assertTrue(log.xml().contains("<Control ID=\"Sequence\"/>"), "built-in models included");
        TreeSpec spec = BtXmlParser.parse(log.xml().replace(
                "<root BTCPP_format=\"4\">",
                "<root BTCPP_format=\"4\" main_tree_to_execute=\"Demo\">"));
        NodeSpec root = spec.mainTree();
        assertEquals("root", root.name());

        List<String> got = new ArrayList<>();
        for (long[] r : log.records()) {
            got.add(r[0] + " " + r[1] + " " + r[2]);
        }
        assertEquals(
                List.of(
                        "0 1 1", // root RUNNING
                        "0 2 2", // AlwaysSuccess SUCCESS (mid-tick, invisible in Status)
                        "0 3 1", // SubTree RUNNING
                        "0 4 1", // Sleep RUNNING
                        "600000 4 2", // Sleep SUCCESS 0.6 s later
                        "600000 4 0", // reset by SubTree
                        "600000 3 2",
                        "600000 2 0",
                        "600000 3 0",
                        "600000 1 2", // root SUCCESS
                        "600000 1 0"), // reset when the command ends
                got);
    }

    @Test
    void recordsReachDiskWhileRunning(@TempDir Path dir) throws Exception {
        BehaviorTreeFileLogger.enable(dir);
        BehaviorTreeCommand cmd = command();
        cmd.initialize();
        cmd.execute(); // 4 transitions, tree still running
        Thread.sleep(4 * BehaviorTreeFileLogger.FLUSH_PERIOD_MS);
        Path file;
        try (var files = Files.list(dir)) {
            file = files.findFirst().orElseThrow();
        }
        assertEquals(4, BtLog.read(file).records().size(), "flushed before the run ended");
        cmd.end(true);
        BehaviorTreeFileLogger.active().orElseThrow().lastWrite().get(5, TimeUnit.SECONDS);
        assertTrue(BtLog.read(file).records().size() > 4, "halt transitions appended at the end");
    }

    @Test
    void noLoggerNoFile(@TempDir Path dir) throws Exception {
        BehaviorTreeCommand cmd = command();
        cmd.initialize();
        cmd.execute();
        cmd.end(true);
        try (var files = Files.list(dir)) {
            assertEquals(0, files.count());
        }
    }
}
