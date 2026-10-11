package com.marswars.bt.monitor;

import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.NodeStatus;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/**
 * A behavior tree as a WPILib {@link Command}, for putting a tree inside a command composition.
 * Every {@code initialize()} starts a fresh run of a {@link BehaviorTreeRunner}, {@code execute()}
 * ticks it once per loop, the command finishes when the root completes, and {@code end()} halts
 * every running node (so {@code cancelAll()} in {@code teleopInit} or disabling stops it cleanly).
 *
 * <p>Autos don't need this: {@code BehaviorTreeAuto} runs its tree straight from the robot loop
 * through the {@code AutoManager}. Its {@code asCommand()} returns one of these.
 */
public class BehaviorTreeCommand extends Command {
    private final BehaviorTreeRunner runner_;

    public BehaviorTreeCommand(String name, Supplier<BehaviorTree> supplier) {
        this(name, supplier, new BehaviorTreeMonitor(name));
    }

    public BehaviorTreeCommand(
            String name, Supplier<BehaviorTree> supplier, BehaviorTreeMonitor monitor) {
        runner_ = new BehaviorTreeRunner(name, supplier, monitor);
        setName(name);
    }

    /** The tree of the current (or last) run, or null before the first run / after a build error. */
    public BehaviorTree getTree() {
        return runner_.getTree();
    }

    public NodeStatus getLastStatus() {
        return runner_.getLastStatus();
    }

    @Override
    public void initialize() {
        runner_.start();
    }

    @Override
    public void execute() {
        runner_.tick();
    }

    @Override
    public boolean isFinished() {
        return !runner_.isRunning();
    }

    @Override
    public void end(boolean interrupted) {
        runner_.stop(interrupted);
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
