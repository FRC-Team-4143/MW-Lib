package com.marswars.bt;

import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.NodeOrigin;
import com.marswars.bt.swerve.SwerveNodes;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.EnumSet;

/**
 * MW-Lib's shared node library. The node-spec of every MW-Lib node (core extensions such as
 * {@code ParallelDeadline} plus every optional bundle such as {@link SwerveNodes}) ships in the jar
 * as {@value #NODE_SPEC_RESOURCE}. A robot's own node-spec file is that same MW-Lib section with
 * the robot's nodes appended ({@code factory.writeTreeNodesModelXml(false)}).
 */
public final class MwLibNodes {
    /** Classpath location of MW-Lib's node-spec ({@code <TreeNodesModel>}) file. */
    public static final String NODE_SPEC_RESOURCE = "/com/marswars/bt/mwlib_nodes.xml";

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

    /** The node-spec file bundled in the jar ({@value #NODE_SPEC_RESOURCE}). */
    public static String bundledNodeSpecXml() {
        try (InputStream in = MwLibNodes.class.getResourceAsStream(NODE_SPEC_RESOURCE)) {
            if (in == null) {
                throw new IllegalStateException(NODE_SPEC_RESOURCE + " is missing from the jar");
            }
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new IllegalStateException("cannot read " + NODE_SPEC_RESOURCE, e);
        }
    }
}
