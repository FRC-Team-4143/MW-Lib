package com.marswars.bt.debug;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.JsonPrimitive;
import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.TreeNode;
import com.marswars.bt.monitor.TreeStructure;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

/**
 * Streams a running behavior tree to editors over WebSocket using the <b>btlive v1</b> protocol
 * (JSON text frames, one object per frame, {@code "op"} discriminator). Java port of the
 * BehaviorTree.CPP {@code bt_live::BtLiveServer}.
 *
 * <ul>
 *   <li>On connect: {@code hello}, then {@code tree} + {@code snapshot} if a tree is attached.
 *   <li>{@link #attach(BehaviorTree)}: new {@code session}, broadcasts {@code tree} + {@code
 *       snapshot}.
 *   <li>Status transitions are recorded on the robot thread and broadcast as batched {@code status}
 *       messages every {@link Options#flushPeriodMs()} by a background thread.
 *   <li>Requests: {@code get_tree}, {@code get_snapshot}, {@code get_blackboard} (optional {@code
 *       request_id}, echoed back); anything else gets an {@code error}.
 * </ul>
 *
 * <p>Timestamps are robot wall-clock milliseconds since the Unix epoch. The server is a debug aid:
 * it never affects tree execution and is not part of log replay (use {@code BehaviorTreeMonitor}'s
 * logged keys for that).
 *
 * <p>Robot code usually calls {@link #start(Options)} once; {@code BehaviorTreeCommand} then
 * attaches every tree it builds to the {@link #active()} server.
 */
public final class BtLiveServer implements AutoCloseable {
    public static final int PROTOCOL_VERSION = 1;
    public static final int DEFAULT_PORT = 1670;

    /**
     * @param host address to bind ({@code "0.0.0.0"} for every interface)
     * @param port TCP port ({@code 0} picks a free one, see {@link #port()})
     * @param robotName display name sent in {@code hello}
     * @param flushPeriodMs how often batched {@code status} messages are sent
     */
    public record Options(String host, int port, String robotName, long flushPeriodMs) {
        public Options {
            host = host == null || host.isEmpty() ? "0.0.0.0" : host;
            robotName = robotName == null ? "" : robotName;
            if (flushPeriodMs <= 0) {
                throw new IllegalArgumentException("flushPeriodMs must be > 0");
            }
        }

        /** {@code 0.0.0.0:1670}, no robot name, 20 ms batches. */
        public static Options defaults() {
            return new Options("0.0.0.0", DEFAULT_PORT, "", 20);
        }

        public Options withPort(int newPort) {
            return new Options(host, newPort, robotName, flushPeriodMs);
        }

        public Options withRobotName(String name) {
            return new Options(host, port, name, flushPeriodMs);
        }

        public Options withHost(String newHost) {
            return new Options(newHost, port, robotName, flushPeriodMs);
        }

        public Options withFlushPeriodMs(long ms) {
            return new Options(host, port, robotName, ms);
        }
    }

    private record Change(double t, int uid, NodeStatus prev, NodeStatus status) {}

    private static final AtomicReference<BtLiveServer> ACTIVE = new AtomicReference<>();

    private final Options options_;
    private final Server server_;
    private final ScheduledExecutorService flusher_;
    private final TreeNode.StatusListener recorder_ = this::record;

    // Guards tree_ and tree_message_.
    private final Object tree_lock_ = new Object();
    private BehaviorTree tree_ = null;
    private String tree_message_ = "";

    // Guards session_, statuses_, pending_. Taken on the robot thread, so kept short.
    private final Object status_lock_ = new Object();
    private long session_ = 0;
    private final Map<Integer, NodeStatus> statuses_ = new HashMap<>();
    private List<Change> pending_ = new ArrayList<>();

    /** Starts a server and makes it the {@link #active()} one. */
    public static BtLiveServer start(Options options) {
        BtLiveServer server = new BtLiveServer(options);
        setActive(server);
        return server;
    }

    /** The server {@code BehaviorTreeCommand} attaches trees to, if one was started. */
    public static Optional<BtLiveServer> active() {
        return Optional.ofNullable(ACTIVE.get());
    }

    public static void setActive(BtLiveServer server) {
        ACTIVE.set(server);
    }

    /**
     * Binds and starts serving. Throws {@link IllegalStateException} when the port cannot be
     * bound.
     */
    public BtLiveServer(Options options) {
        options_ = Objects.requireNonNull(options, "options");
        server_ = new Server(new InetSocketAddress(options.host(), options.port()));
        server_.setReuseAddr(true);
        server_.setDaemon(true);
        server_.start();
        try {
            if (!server_.started_.await(5, TimeUnit.SECONDS)) {
                throw new IllegalStateException("bt_live: server did not start in time");
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("bt_live: interrupted while starting", e);
        }
        if (server_.start_error_ != null) {
            stopQuietly();
            throw new IllegalStateException(
                    "bt_live: cannot listen on " + options.host() + ":" + options.port(),
                    server_.start_error_);
        }
        flusher_ =
                Executors.newSingleThreadScheduledExecutor(
                        r -> {
                            Thread t = new Thread(r, "bt_live-flusher");
                            t.setDaemon(true);
                            return t;
                        });
        flusher_.scheduleAtFixedRate(
                this::flush, options.flushPeriodMs(), options.flushPeriodMs(), TimeUnit.MILLISECONDS);
    }

    public Options options() {
        return options_;
    }

    /** Bound port (useful with port 0). */
    public int port() {
        return server_.getPort();
    }

    public int clientCount() {
        return server_.getConnections().size();
    }

    // ------------------------------------------------------------------ application API

    /** Streams {@code tree} from now on (replacing any previous tree) under a new session. */
    public void attach(BehaviorTree tree) {
        Objects.requireNonNull(tree, "tree");
        String tree_msg;
        String snapshot_msg;
        synchronized (tree_lock_) {
            if (tree_ != null) {
                tree_.removeStatusListener(recorder_);
            }
            tree_ = tree;
            long session;
            synchronized (status_lock_) {
                session = ++session_;
                pending_ = new ArrayList<>();
                statuses_.clear();
                for (TreeNode node : tree.getNodes()) {
                    statuses_.put(node.getUid(), node.getStatus());
                }
            }
            JsonObject msg = new JsonObject();
            msg.addProperty("op", "tree");
            msg.addProperty("session", session);
            msg.addProperty("tree_id", tree.getMainTreeId());
            msg.add("nodes", TreeStructure.nodes(tree));
            msg.addProperty("xml", tree.getXml().orElse(""));
            tree_message_ = msg.toString();
            tree_msg = tree_message_;
            tree.addStatusListener(recorder_);
            snapshot_msg = snapshotMessage();
        }
        server_.broadcast(tree_msg);
        server_.broadcast(snapshot_msg);
    }

    /** Stops streaming; clients connecting from now on get no tree until the next attach. */
    public void detach() {
        synchronized (tree_lock_) {
            if (tree_ != null) {
                tree_.removeStatusListener(recorder_);
            }
            tree_ = null;
            tree_message_ = "";
        }
    }

    /** Broadcasts an application error (e.g. an exception thrown while ticking). */
    public void reportError(String message) {
        JsonObject msg = new JsonObject();
        msg.addProperty("op", "error");
        msg.addProperty("message", message);
        server_.broadcast(msg.toString());
    }

    /** Stops the server and the flusher; clears {@link #active()} if it was this server. */
    @Override
    public void close() {
        detach();
        flusher_.shutdownNow();
        ACTIVE.compareAndSet(this, null);
        stopQuietly();
    }

    private void stopQuietly() {
        try {
            server_.stop(1000, "server shutting down");
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /** BT.CPP {@code writeTreeNodesModelXML(factory, false)}: the editor's palette document. */
    public static String nodeSpecXml(BehaviorTreeFactory factory) {
        return factory.writeTreeNodesModelXml(false);
    }

    public static void writeNodeSpec(BehaviorTreeFactory factory, Path path) {
        try {
            Files.writeString(path, nodeSpecXml(factory), StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new IllegalStateException("bt_live: cannot write node spec to " + path, e);
        }
    }

    // ------------------------------------------------------------------ status recording

    private static double nowMs() {
        Instant now = Instant.now();
        return now.getEpochSecond() * 1000.0 + now.getNano() / 1.0e6;
    }

    private void record(TreeNode node, NodeStatus prev, NodeStatus status, double treeTime) {
        synchronized (status_lock_) {
            statuses_.put(node.getUid(), status);
            pending_.add(new Change(nowMs(), node.getUid(), prev, status));
        }
    }

    private void flush() {
        List<Change> changes;
        long session;
        synchronized (status_lock_) {
            if (pending_.isEmpty()) {
                return;
            }
            changes = pending_;
            pending_ = new ArrayList<>();
            session = session_;
        }
        JsonArray rows = new JsonArray();
        for (Change c : changes) {
            JsonArray row = new JsonArray();
            row.add(c.t());
            row.add(c.uid());
            row.add(c.prev().name());
            row.add(c.status().name());
            rows.add(row);
        }
        JsonObject msg = new JsonObject();
        msg.addProperty("op", "status");
        msg.addProperty("session", session);
        msg.add("changes", rows);
        try {
            server_.broadcast(msg.toString());
        } catch (RuntimeException e) {
            // never let a flaky client kill the flusher
        }
    }

    // ------------------------------------------------------------------ messages

    private String helloMessage() {
        JsonObject msg = new JsonObject();
        msg.addProperty("op", "hello");
        msg.addProperty("protocol", "btlive");
        msg.addProperty("version", PROTOCOL_VERSION);
        msg.addProperty("robot", options_.robotName());
        return msg.toString();
    }

    private String snapshotMessage() {
        JsonArray entries = new JsonArray();
        long session;
        synchronized (status_lock_) {
            session = session_;
            statuses_.entrySet().stream()
                    .filter(e -> e.getValue() != NodeStatus.IDLE)
                    .sorted(Map.Entry.comparingByKey())
                    .forEach(
                            e -> {
                                JsonArray row = new JsonArray();
                                row.add(e.getKey());
                                row.add(e.getValue().name());
                                entries.add(row);
                            });
        }
        JsonObject msg = new JsonObject();
        msg.addProperty("op", "snapshot");
        msg.addProperty("session", session);
        msg.addProperty("t", nowMs());
        msg.add("statuses", entries);
        return msg.toString();
    }

    private String blackboardMessage(JsonElement requestId) {
        JsonArray boards = new JsonArray();
        synchronized (tree_lock_) {
            if (tree_ != null) {
                for (BehaviorTree.Subtree subtree : tree_.getSubtrees()) {
                    JsonObject board = new JsonObject();
                    board.addProperty(
                            "path",
                            subtree.instanceName().isEmpty()
                                    ? subtree.treeId()
                                    : subtree.instanceName());
                    board.addProperty("tree_id", subtree.treeId());
                    board.add("entries", BlackboardJson.entries(subtree.blackboard()));
                    boards.add(board);
                }
            }
        }
        JsonObject msg = new JsonObject();
        msg.addProperty("op", "blackboard");
        if (requestId != null && !requestId.isJsonNull()) {
            msg.add("request_id", requestId);
        }
        synchronized (status_lock_) {
            msg.addProperty("session", session_);
        }
        msg.add("blackboards", boards);
        return msg.toString();
    }

    private static String errorMessage(String message, JsonElement requestId) {
        JsonObject msg = new JsonObject();
        msg.addProperty("op", "error");
        msg.addProperty("message", message);
        if (requestId != null && !requestId.isJsonNull()) {
            msg.add("request_id", requestId);
        }
        return msg.toString();
    }

    private void handleRequest(WebSocket conn, String text) {
        JsonElement parsed;
        try {
            parsed = JsonParser.parseString(text);
        } catch (RuntimeException e) {
            parsed = JsonNull.INSTANCE;
        }
        if (!parsed.isJsonObject()
                || !parsed.getAsJsonObject().has("op")
                || !(parsed.getAsJsonObject().get("op") instanceof JsonPrimitive p
                        && p.isString())) {
            conn.send(errorMessage("expected a JSON object with an \"op\"", null));
            return;
        }
        JsonObject request = parsed.getAsJsonObject();
        String op = request.get("op").getAsString();
        JsonElement request_id = request.get("request_id");
        switch (op) {
            case "get_tree" -> {
                String msg;
                synchronized (tree_lock_) {
                    msg = tree_message_;
                }
                if (!msg.isEmpty()) {
                    conn.send(msg);
                }
            }
            case "get_snapshot" -> conn.send(snapshotMessage());
            case "get_blackboard" -> conn.send(blackboardMessage(request_id));
            default -> conn.send(errorMessage("unknown op \"" + op + "\"", request_id));
        }
    }

    // ------------------------------------------------------------------ socket server

    private final class Server extends WebSocketServer {
        final CountDownLatch started_ = new CountDownLatch(1);
        volatile Exception start_error_ = null;

        Server(InetSocketAddress address) {
            super(address);
        }

        @Override
        public void onStart() {
            started_.countDown();
        }

        @Override
        public void onOpen(WebSocket conn, ClientHandshake handshake) {
            conn.send(helloMessage());
            String tree_msg;
            synchronized (tree_lock_) {
                tree_msg = tree_message_;
            }
            if (!tree_msg.isEmpty()) {
                conn.send(tree_msg);
                conn.send(snapshotMessage());
            }
        }

        @Override
        public void onMessage(WebSocket conn, String message) {
            handleRequest(conn, message);
        }

        @Override
        public void onClose(WebSocket conn, int code, String reason, boolean remote) {}

        @Override
        public void onError(WebSocket conn, Exception ex) {
            if (conn == null && started_.getCount() > 0) {
                start_error_ = ex; // bind failure
                started_.countDown();
            }
        }
    }
}
