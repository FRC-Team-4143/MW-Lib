package com.marswars.bt;

import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.NodeOrigin;
import com.marswars.bt.swerve.SwerveNodes;
import java.util.EnumSet;

/**
 * MW-Lib's shared node library. {@link #nodeSpecXml()} generates the node-spec of every MW-Lib node
 * (core extensions such as {@code ParallelDeadline} plus every optional bundle such as {@link
 * SwerveNodes}) from the registrations themselves, so there is no file to keep in sync. A robot's
 * own node-spec file (the editor's palette) is that same MW-Lib section with the robot's nodes
 * appended ({@code factory.writeTreeNodesModelXml(false)}).
 */
public final class MwLibNodes {
    private MwLibNodes() {}

    /**
     * Factory with every MW-Lib node registered, for documentation and node-spec generation. The
     * subsystem suppliers throw if a tree from this factory is ever ticked.
     */
    public static BehaviorTreeFactory libraryFactory() {
        BehaviorTreeFactory factory = new BehaviorTreeFactory(() -> 0.0);
        SwerveNodes.register(
                factory,
                () -> {
                    throw new IllegalStateException("MwLibNodes.libraryFactory() is for docs only");
                });
        return factory;
    }

    /** Freshly generated node-spec of every MW-Lib node. */
    public static String nodeSpecXml() {
        return libraryFactory().writeTreeNodesModelXml(EnumSet.of(NodeOrigin.MWLIB));
    }
}
