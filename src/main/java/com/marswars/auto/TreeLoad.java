package com.marswars.auto;

import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.Blackboard;
import com.marswars.bt.core.TreeSpec;
import com.marswars.bt.xml.BtXmlParser;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;

/**
 * One load of an auto's XML: the text, the parsed tree (null when the load failed, with {@code
 * error} saying why), the trajectories it names and, when loaded, those trajectories
 * alliance-flipped. Built without touching any shared state, so it can be made on a background
 * thread and handed to the robot thread.
 */
record TreeLoad(
        String xml,
        TreeSpec spec,
        String error,
        List<String> trajectoryNames,
        Map<String, ChoreoTrajectory> trajectories) {

    boolean ok() {
        return spec != null;
    }

    static TreeLoad failed(String xml, String error) {
        return new TreeLoad(xml, null, error, List.of(), null);
    }

    /**
     * Parses {@code xml} and checks it the way a run would use it: builds the tree once (unknown
     * nodes, bad ports, wrong child counts) and checks every {@code {key}} it reads has a source,
     * given the keys a run's {@code root} starts with. Then finds its trajectory names and, when
     * {@code is_red_alliance} is not null, loads and flips them.
     *
     * @param base_dir directory {@code <include>} paths resolve against (may be null)
     */
    static TreeLoad of(
            String xml,
            Path base_dir,
            BehaviorTreeFactory factory,
            Blackboard root,
            Boolean is_red_alliance) {
        if (xml == null) {
            return failed(null, "no XML");
        }
        try {
            TreeSpec spec = BtXmlParser.parse(xml, base_dir);
            factory.createTree(spec, root, xml);
            factory.checkBlackboardFlow(spec, root);
            List<String> names = TrajectoryNames.discover(spec, factory);
            Map<String, ChoreoTrajectory> loaded =
                    is_red_alliance == null ? null : AutoTrajectories.load(names, is_red_alliance);
            return new TreeLoad(xml, spec, "", names, loaded);
        } catch (RuntimeException e) {
            return failed(xml, e.getMessage());
        }
    }
}
