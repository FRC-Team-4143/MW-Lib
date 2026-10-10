package com.marswars.bt.core;

/**
 * Where a node type comes from. Node-spec files ({@code <TreeNodesModel>}) for editors list every
 * origin except {@link #BTCPP}, which editors already know, grouped in this order: MW-Lib's shared
 * nodes first, then the robot's own nodes appended after them.
 */
public enum NodeOrigin {
    /** BehaviorTree.CPP v4 built-ins (Sequence, Parallel, Sleep, ...). */
    BTCPP("BehaviorTree.CPP built-in nodes"),
    /** Generic nodes shipped by MW-Lib and shared by every robot (ParallelDeadline, swerve, ...). */
    MWLIB("MW-Lib shared nodes (com.marswars.bt)"),
    /** Nodes registered by robot code. */
    ROBOT("Robot nodes"),
    /** Declared in an XML file's <TreeNodesModel> only (not registered in Java). */
    FILE("Declared in XML");

    private final String title_;

    NodeOrigin(String title) {
        title_ = title;
    }

    /** Section title used as a comment in generated node-spec files. */
    public String title() {
        return title_;
    }
}
