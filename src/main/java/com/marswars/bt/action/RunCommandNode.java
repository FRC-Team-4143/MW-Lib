package com.marswars.bt.action;

import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.StatefulActionNode;
import com.marswars.bt.core.TreeNode;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Objects;
import java.util.function.Function;

/**
 * Runs a WPILib {@link Command} as a leaf, driven directly by the tree (not scheduled): start calls
 * {@code initialize()}, each tick calls {@code execute()} until {@code isFinished()}, then {@code
 * end(false)} and SUCCESS; a halt calls {@code end(true)}. Commands are created fresh each start.
 *
 * <p>The command is not given to the {@code CommandScheduler}, so its requirements are not
 * enforced; MW-Lib subsystems do not use requirements anyway.
 */
public class RunCommandNode extends StatefulActionNode {
    private final Function<TreeNode, Command> factory_;
    private Command command_ = null;

    public RunCommandNode(String name, NodeConfig config, Function<TreeNode, Command> factory) {
        super(name, config);
        factory_ = Objects.requireNonNull(factory, "factory");
    }

    @Override
    protected NodeStatus onStart() {
        command_ = factory_.apply(this);
        command_.initialize();
        return onRunning();
    }

    @Override
    protected NodeStatus onRunning() {
        command_.execute();
        if (command_.isFinished()) {
            command_.end(false);
            command_ = null;
            return NodeStatus.SUCCESS;
        }
        return NodeStatus.RUNNING;
    }

    @Override
    protected void onHalted() {
        if (command_ != null) {
            command_.end(true);
            command_ = null;
        }
    }
}
