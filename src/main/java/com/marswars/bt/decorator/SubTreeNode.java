package com.marswars.bt.decorator;

import com.marswars.bt.core.DecoratorNode;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeKind;
import com.marswars.bt.core.NodeStatus;

/**
 * Instance of another {@code <BehaviorTree>}: a decorator whose single child is the root of the
 * instantiated tree, which uses its own (remapped) blackboard. BT.CPP {@code SubTree}.
 */
public class SubTreeNode extends DecoratorNode {
    private final String subtree_id_;

    public SubTreeNode(String name, NodeConfig config, String subtreeId) {
        super(name, config);
        subtree_id_ = subtreeId;
    }

    @Override
    protected String defaultRegistrationId() {
        return "SubTree";
    }

    @Override
    public NodeKind kind() {
        return NodeKind.SUBTREE;
    }

    /** ID of the {@code <BehaviorTree>} this node instantiates. */
    public String subtreeId() {
        return subtree_id_;
    }

    @Override
    protected NodeStatus tick() {
        if (getStatus() == NodeStatus.IDLE) {
            setStatus(NodeStatus.RUNNING);
        }
        NodeStatus child_status = child().executeTick();
        if (child_status.isCompleted()) {
            resetChild();
        }
        return child_status;
    }
}
