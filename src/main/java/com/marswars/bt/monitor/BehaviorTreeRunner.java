package com.marswars.bt.monitor;

import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.debug.BtLiveServer;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Runs one behavior tree, run after run, with no WPILib command involved: {@link #start()} builds a
 * fresh tree from the supplier (so per-run node state never leaks between enables), {@link #tick()}
 * ticks it once and finishes the run when the root completes, and {@link #stop} halts every running
 * node. {@code BehaviorTreeAuto} calls these from the robot loop; {@link BehaviorTreeCommand} wraps
 * the same runner for a tree inside a command composition.
 *
 * <p>Each run is logged through a {@link BehaviorTreeMonitor}, attached to the active {@link
 * BtLiveServer} (if any), and recorded by the active {@link BehaviorTreeFileLogger} (if any). The
 * clock is the tree factory's (replay-safe by default).
 */
public final class BehaviorTreeRunner {
    /** Where error text goes besides the monitor and live server; replaceable for tests. */
    static Consumer<String> message_log_ = DataLogManager::log;

    private final String name_;
    private final Supplier<BehaviorTree> supplier_;
    private final BehaviorTreeMonitor monitor_;
    private BehaviorTree tree_ = null;
    private BehaviorTreeFileLogger.Run file_log_ = null;
    private NodeStatus last_status_ = NodeStatus.IDLE;
    private boolean running_ = false;

    public BehaviorTreeRunner(String name, Supplier<BehaviorTree> supplier, BehaviorTreeMonitor monitor) {
        name_ = Objects.requireNonNull(name, "name");
        supplier_ = Objects.requireNonNull(supplier, "supplier");
        monitor_ = Objects.requireNonNull(monitor, "monitor");
    }

    public String getName() {
        return name_;
    }

    /** The tree of the current (or last) run, or null before the first run / after a build error. */
    public BehaviorTree getTree() {
        return tree_;
    }

    public NodeStatus getLastStatus() {
        return last_status_;
    }

    /** True from {@link #start()} until the root completes or {@link #stop} is called. */
    public boolean isRunning() {
        return running_;
    }

    /** Starts a run: a fresh tree, logging, live view and run log. Stops any run in progress first. */
    public void start() {
        stop(true);
        last_status_ = NodeStatus.IDLE;
        try {
            tree_ = supplier_.get();
        } catch (RuntimeException e) {
            tree_ = null;
            reportError("cannot build tree: " + e.getMessage());
            monitor_.publishResult("ERROR");
            return;
        }
        running_ = true;
        monitor_.publishError("");
        monitor_.publishStart(tree_);
        monitor_.publishActive(name_);
        BtLiveServer.active().ifPresent(server -> server.attach(tree_));
        file_log_ = BehaviorTreeFileLogger.active().map(l -> l.start(name_, tree_)).orElse(null);
    }

    /** Ticks the tree once; when the root completes, the run is finished and logged. */
    public void tick() {
        if (!running_) {
            return;
        }
        try {
            last_status_ = tree_.tickOnce();
        } catch (RuntimeException e) {
            last_status_ = NodeStatus.FAILURE;
            reportError("exception while ticking: " + e);
        }
        monitor_.publishTransitions();
        if (last_status_.isCompleted() || last_status_ == NodeStatus.SKIPPED) {
            finish(last_status_.name());
        }
    }

    /** Ends the run early, halting every running node. No-op when not running. */
    public void stop(boolean interrupted) {
        if (running_) {
            finish(interrupted ? "INTERRUPTED" : last_status_.name());
        }
    }

    private void finish(String result) {
        running_ = false;
        tree_.haltTree();
        monitor_.publishTransitions();
        monitor_.detach();
        monitor_.publishResult(result);
        if (file_log_ != null) {
            file_log_.finish(result);
            file_log_ = null;
        }
        monitor_.publishActive("");
    }

    private void reportError(String message) {
        String full = name_ + ": " + message;
        monitor_.publishError(message);
        message_log_.accept("[BehaviorTree] " + full);
        BtLiveServer.active().ifPresent(server -> server.reportError(full));
    }
}
