package com.marswars.bt.debug;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.support.FakeClock;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.WebSocket;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CompletionStage;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.function.Predicate;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class BtLiveServerTest {
    static final String XML =
            "<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">"
                    + "<BehaviorTree ID=\"MainTree\"><Sequence name=\"root\">"
                    + "<SetBlackboard value=\"3\" output_key=\"count\"/>"
                    + "<SubTree ID=\"Child\" x=\"{count}\"/>"
                    + "<Sleep msec=\"1000\"/></Sequence></BehaviorTree>"
                    + "<BehaviorTree ID=\"Child\"><SetBlackboard value=\"hi\" output_key=\"local\"/>"
                    + "</BehaviorTree></root>";

    final FakeClock clock = new FakeClock();
    BtLiveServer server;
    final List<Client> clients = new ArrayList<>();

    /** Minimal JDK WebSocket client that queues every text frame as JSON. */
    static final class Client implements WebSocket.Listener {
        final BlockingQueue<JsonObject> frames = new LinkedBlockingQueue<>();
        final StringBuilder partial = new StringBuilder();
        WebSocket ws;

        static Client connect(int port) throws Exception {
            Client c = new Client();
            c.ws =
                    HttpClient.newHttpClient()
                            .newWebSocketBuilder()
                            .buildAsync(URI.create("ws://127.0.0.1:" + port + "/"), c)
                            .get(5, TimeUnit.SECONDS);
            return c;
        }

        @Override
        public CompletionStage<?> onText(WebSocket webSocket, CharSequence data, boolean last) {
            partial.append(data);
            if (last) {
                frames.add(JsonParser.parseString(partial.toString()).getAsJsonObject());
                partial.setLength(0);
            }
            webSocket.request(1);
            return null;
        }

        void send(String json) throws Exception {
            ws.sendText(json, true).get(5, TimeUnit.SECONDS);
        }

        JsonObject next() throws InterruptedException {
            JsonObject o = frames.poll(5, TimeUnit.SECONDS);
            assertNotNull(o, "timed out waiting for a frame");
            return o;
        }

        JsonObject nextOp(String op) throws InterruptedException {
            return next(o -> op.equals(o.get("op").getAsString()));
        }

        JsonObject next(Predicate<JsonObject> match) throws InterruptedException {
            while (true) {
                JsonObject o = next();
                if (match.test(o)) {
                    return o;
                }
            }
        }
    }

    @BeforeEach
    void setUp() {
        server =
                new BtLiveServer(
                        BtLiveServer.Options.defaults()
                                .withHost("127.0.0.1")
                                .withPort(0)
                                .withRobotName("testbot")
                                .withFlushPeriodMs(10));
    }

    @AfterEach
    void tearDown() {
        for (Client c : clients) {
            c.ws.abort();
        }
        server.close();
    }

    Client client() throws Exception {
        Client c = Client.connect(server.port());
        clients.add(c);
        return c;
    }

    @Test
    void helloOnlyWhenNothingAttached() throws Exception {
        Client c = client();
        JsonObject hello = c.next();
        assertEquals("hello", hello.get("op").getAsString());
        assertEquals("btlive", hello.get("protocol").getAsString());
        assertEquals(1, hello.get("version").getAsInt());
        assertEquals("testbot", hello.get("robot").getAsString());
        assertTrue(c.frames.poll(100, TimeUnit.MILLISECONDS) == null, "no tree before attach");
    }

    @Test
    void attachBroadcastsTreeSnapshotAndBatchedStatus() throws Exception {
        Client c = client();
        c.nextOp("hello");
        BehaviorTree tree = new BehaviorTreeFactory(clock).createTreeFromText(XML);
        server.attach(tree);

        JsonObject treeMsg = c.nextOp("tree");
        long session = treeMsg.get("session").getAsLong();
        assertEquals("MainTree", treeMsg.get("tree_id").getAsString());
        assertEquals(XML, treeMsg.get("xml").getAsString());
        JsonArray nodes = treeMsg.getAsJsonArray("nodes");
        assertEquals(5, nodes.size());
        JsonObject sub = nodes.get(2).getAsJsonObject();
        assertEquals("SubTree", sub.get("type").getAsString());
        assertEquals("Child", sub.get("subtree").getAsString());
        assertEquals("SubTree", sub.get("category").getAsString());
        assertEquals("Child::3", sub.get("path").getAsString());
        assertEquals(
                "Child::3/SetBlackboard::4",
                nodes.get(3).getAsJsonObject().get("path").getAsString());
        assertEquals(3, nodes.get(3).getAsJsonObject().get("parent").getAsInt());

        JsonObject snap = c.nextOp("snapshot");
        assertEquals(session, snap.get("session").getAsLong());
        assertEquals(0, snap.getAsJsonArray("statuses").size());
        assertTrue(snap.get("t").getAsDouble() > 1.7e12, "wall clock ms");

        tree.tickOnce();
        List<String> changes = new ArrayList<>();
        while (changes.size() < 6) {
            JsonObject status = c.nextOp("status");
            assertEquals(session, status.get("session").getAsLong());
            for (JsonElement row : status.getAsJsonArray("changes")) {
                JsonArray r = row.getAsJsonArray();
                assertEquals(4, r.size());
                changes.add(r.get(1).getAsInt() + ":" + r.get(2).getAsString() + ">" + r.get(3).getAsString());
            }
        }
        assertEquals(
                List.of(
                        "1:IDLE>RUNNING",
                        "2:IDLE>SUCCESS",
                        "3:IDLE>RUNNING",
                        "4:IDLE>SUCCESS",
                        "4:SUCCESS>IDLE",
                        "3:RUNNING>SUCCESS"),
                changes.subList(0, 6));

        c.send("{\"op\":\"get_snapshot\"}");
        JsonObject snap2 = c.nextOp("snapshot");
        assertEquals("[[1,\"RUNNING\"],[2,\"SUCCESS\"],[3,\"SUCCESS\"],[5,\"RUNNING\"]]",
                snap2.getAsJsonArray("statuses").toString());
    }

    @Test
    void lateClientGetsTreeAndSnapshotAndNewSessionOnReattach() throws Exception {
        BehaviorTreeFactory factory = new BehaviorTreeFactory(clock);
        BehaviorTree tree = factory.createTreeFromText(XML);
        server.attach(tree);
        tree.tickOnce();

        Client c = client();
        c.nextOp("hello");
        long s1 = c.nextOp("tree").get("session").getAsLong();
        JsonObject snap = c.nextOp("snapshot");
        assertEquals(s1, snap.get("session").getAsLong());
        assertFalse(snap.getAsJsonArray("statuses").isEmpty());

        server.attach(factory.createTreeFromText(XML));
        long s2 = c.nextOp("tree").get("session").getAsLong();
        assertEquals(s1 + 1, s2);

        tree.tickOnce(); // old tree: must not be streamed any more
        c.send("{\"op\":\"get_tree\"}");
        assertEquals(s2, c.nextOp("tree").get("session").getAsLong());
        for (JsonObject f : c.frames) {
            if ("status".equals(f.get("op").getAsString())) {
                assertEquals(s2, f.get("session").getAsLong());
            }
        }
    }

    @Test
    void blackboardRequestEchoesRequestId() throws Exception {
        BehaviorTree tree = new BehaviorTreeFactory(clock).createTreeFromText(XML);
        server.attach(tree);
        tree.tickOnce();
        Client c = client();
        c.send("{\"op\":\"get_blackboard\",\"request_id\":7}");
        JsonObject bb = c.nextOp("blackboard");
        assertEquals(7, bb.get("request_id").getAsInt());
        JsonArray boards = bb.getAsJsonArray("blackboards");
        assertEquals(2, boards.size());
        JsonObject main = boards.get(0).getAsJsonObject();
        assertEquals("MainTree", main.get("path").getAsString());
        JsonObject count = main.getAsJsonObject("entries").getAsJsonObject("count");
        assertEquals("std::string", count.get("type").getAsString());
        assertEquals("3", count.get("value").getAsString());
        JsonObject child = boards.get(1).getAsJsonObject();
        assertEquals("Child::3", child.get("path").getAsString());
        assertEquals("Child", child.get("tree_id").getAsString());
        assertEquals("hi", child.getAsJsonObject("entries").getAsJsonObject("local").get("value").getAsString());
    }

    @Test
    void badRequestsGetErrors() throws Exception {
        Client c = client();
        c.nextOp("hello");
        c.send("not json");
        assertEquals(
                "expected a JSON object with an \"op\"", c.nextOp("error").get("message").getAsString());
        c.send("{\"op\":\"pause\",\"request_id\":\"abc\"}");
        JsonObject err = c.nextOp("error");
        assertEquals("unknown op \"pause\"", err.get("message").getAsString());
        assertEquals("abc", err.get("request_id").getAsString());

        server.reportError("tick blew up");
        assertEquals("tick blew up", c.nextOp("error").get("message").getAsString());
    }

    @Test
    void detachStopsServingTree() throws Exception {
        server.attach(new BehaviorTreeFactory(clock).createTreeFromText(XML));
        server.detach();
        Client c = client();
        c.nextOp("hello");
        c.send("{\"op\":\"get_tree\"}");
        c.send("{\"op\":\"get_snapshot\"}");
        assertEquals("snapshot", c.next().get("op").getAsString(), "no tree reply after detach");
        assertEquals(1, server.clientCount());
    }

    @Test
    void nodeSpecFileIsBtcppModelXml(@TempDir Path dir) throws Exception {
        BehaviorTreeFactory factory = new BehaviorTreeFactory(clock);
        factory.registerInstantAction("Ping", "says ping", List.of(), n -> {});
        Path file = dir.resolve("nodes.xml");
        BtLiveServer.writeNodeSpec(factory, file);
        String xml = Files.readString(file);
        assertTrue(xml.contains("<Action ID=\"Ping\">"));
        assertFalse(xml.contains("ID=\"Sequence\""));
    }

    @Test
    void activeServerRegistry() {
        assertTrue(BtLiveServer.active().isEmpty() || BtLiveServer.active().get() != server);
        BtLiveServer.setActive(server);
        assertEquals(server, BtLiveServer.active().orElseThrow());
        server.close();
        assertTrue(BtLiveServer.active().isEmpty());
    }
}
