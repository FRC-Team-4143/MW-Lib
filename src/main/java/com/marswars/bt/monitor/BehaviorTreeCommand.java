package com.marswars.bt.monitor;

import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.debug.BtLiveServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Runs a behavior tree as a WPILib {@link Command}: every {@code initialize()} builds a fresh tree
 * from the supplier (so per-run node state never leaks between enables), {@code execute()} ticks
 * it once per loop, the command finishes when the root completes, and {@code end()} halts every
 * running node (so {@code cancelAll()} in {@code teleopInit} or disabling stops the auto cleanly).
 *
 * <p>Ticks happen inside {@code CommandScheduler.run()}, which runs before the MW-Lib subsystem
 * loop, so wanted states requested by nodes are applied in the same robot loop. The clock is the
 * factory's (replay-safe by default).
 *
 * <p>Each new tree is logged through a {@link BehaviorTreeMonitor}, attached to the active {@link
 * BtLiveServer} (if any), and recorded by the active {@link BehaviorTreeFileLogger} (if any).
 */
public class BehaviorTreeCommand extends Command {
    /** Where error text goes besides the monitor and live server; replaceable for tests. */
    static Consumer<String> message_log_ = DataLogManager::log;

    private final Supplier<BehaviorTree> supplier_;
    private final BehaviorTreeMonitor monitor_;
    private BehaviorTree tree_ = null;
    private BehaviorTreeFileLogger.Run file_log_ = null;
    private NodeStatus last_status_ = NodeStatus.IDLE;

    public BehaviorTreeCommand(String name, Supplier<BehaviorTree> supplier) {
        this(name, supplier, new BehaviorTreeMonitor(name));
    }

    public BehaviorTreeCommand(
            String name, Supplier<BehaviorTree> supplier, BehaviorTreeMonitor monitor) {
        supplier_ = Objects.requireNonNull(supplier, "supplier");
        monitor_ = Objects.requireNonNull(monitor, "monitor");
        setName(name);
    }

    /** The tree of the current (or last) run, or null before the first run / after a build error. */
    public BehaviorTree getTree() {
        return tree_;
    }

    public NodeStatus getLastStatus() {
        return last_status_;
    }

    @Override
    public void initialize() {
        last_status_ = NodeStatus.IDLE;
        try {
            tree_ = supplier_.get();
        } catch (RuntimeException e) {
            tree_ = null;
            reportError("cannot build tree: " + e.getMessage());
            return;
        }
        monitor_.publishError("");
        monitor_.publishTree(tree_);
        monitor_.publishActive(getName());
        BtLiveServer.active().ifPresent(server -> server.attach(tree_));
        file_log_ =
                BehaviorTreeFileLogger.active().map(l -> l.start(getName(), tree_)).orElse(null);
    }

    @Override
    public void execute() {
        if (tree_ == null) {
            return;
        }
        try {
            last_status_ = tree_.tickOnce();
        } catch (RuntimeException e) {
            last_status_ = NodeStatus.FAILURE;
            reportError("exception while ticking: " + e);
        }
        monitor_.publishStatus(tree_);
    }

    @Override
    public boolean isFinished() {
        return tree_ == null || last_status_.isCompleted() || last_status_ == NodeStatus.SKIPPED;
    }

    @Override
    public void end(boolean interrupted) {
        if (tree_ == null) {
            monitor_.publishResult("ERROR");
            return;
        }
        tree_.haltTree();
        monitor_.publishStatus(tree_);
        String result = interrupted ? "INTERRUPTED" : last_status_.name();
        monitor_.publishResult(result);
        if (file_log_ != null) {
            file_log_.finish(result);
            file_log_ = null;
        }
        monitor_.publishActive("");
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    private void reportError(String message) {
        String full = getName() + ": " + message;
        monitor_.publishError(message);
        message_log_.accept("[BehaviorTree] " + full);
        BtLiveServer.active().ifPresent(server -> server.reportError(full));
    }
}
