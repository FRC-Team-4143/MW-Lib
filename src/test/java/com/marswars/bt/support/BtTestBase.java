package com.marswars.bt.support;

import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.Blackboard;
import com.marswars.bt.core.ControlNode;
import com.marswars.bt.core.DecoratorNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.TreeNode;
import java.util.Map;

/** Shared helpers for hand-built trees. */
public abstract class BtTestBase {
    protected final FakeClock clock = new FakeClock();
    protected final Blackboard bb = Blackboard.createRoot();

    protected NodeConfig cfg() {
        return NodeConfig.of(bb, clock);
    }

    protected NodeConfig cfg(Map<String, String> inputs) {
        return NodeConfig.of(bb, clock).withInputs(inputs);
    }

    protected ScriptedAction action(String name, NodeStatus... script) {
        return new ScriptedAction(name, cfg(), script);
    }

    protected Flag flag(String name, boolean value) {
        return new Flag(name, cfg(), value);
    }

    protected static <C extends ControlNode> C with(C control, TreeNode... children) {
        for (TreeNode c : children) {
            control.addChild(c);
        }
        return control;
    }

    protected static <D extends DecoratorNode> D with(D decorator, TreeNode child) {
        decorator.setChild(child);
        return decorator;
    }

    protected BehaviorTree tree(TreeNode root) {
        return new BehaviorTree(root, bb, clock);
    }

    protected static final NodeStatus R = NodeStatus.RUNNING;
    protected static final NodeStatus S = NodeStatus.SUCCESS;
    protected static final NodeStatus F = NodeStatus.FAILURE;
    protected static final NodeStatus I = NodeStatus.IDLE;
    protected static final NodeStatus K = NodeStatus.SKIPPED;
}
