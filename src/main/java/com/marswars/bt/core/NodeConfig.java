package com.marswars.bt.core;

import java.util.Map;
import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * Everything a node needs at construction: its blackboard, the raw port strings from the XML (or
 * builder), the tree clock, its path for diagnostics, and the model it was instantiated from.
 *
 * @param blackboard blackboard of the (sub)tree the node lives in
 * @param inputPorts raw attribute strings for input/inout ports (literal or {@code {key}})
 * @param outputPorts raw attribute strings for output/inout ports (must be {@code {key}})
 * @param clock monotonic seconds; replay-safe when wired to {@code MwLog::timestampSeconds}
 * @param path diagnostic path such as {@code MainTree/Sequence/FollowTrajectory}
 * @param model registered model, or {@code null} for hand-built nodes
 */
public record NodeConfig(
        Blackboard blackboard,
        Map<String, String> inputPorts,
        Map<String, String> outputPorts,
        DoubleSupplier clock,
        String path,
        NodeModel model) {

    public NodeConfig {
        Objects.requireNonNull(blackboard, "blackboard");
        Objects.requireNonNull(clock, "clock");
        inputPorts = inputPorts == null ? Map.of() : Map.copyOf(inputPorts);
        outputPorts = outputPorts == null ? Map.of() : Map.copyOf(outputPorts);
        path = path == null ? "" : path;
    }

    /** Minimal config for hand-built trees and tests: no ports, no model. */
    public static NodeConfig of(Blackboard blackboard, DoubleSupplier clock) {
        return new NodeConfig(blackboard, Map.of(), Map.of(), clock, "", null);
    }

    public NodeConfig withPorts(Map<String, String> inputs, Map<String, String> outputs) {
        return new NodeConfig(blackboard, inputs, outputs, clock, path, model);
    }

    public NodeConfig withInputs(Map<String, String> inputs) {
        return new NodeConfig(blackboard, inputs, outputPorts, clock, path, model);
    }

    public NodeConfig withModel(NodeModel newModel) {
        return new NodeConfig(blackboard, inputPorts, outputPorts, clock, path, newModel);
    }

    public NodeConfig withPath(String newPath) {
        return new NodeConfig(blackboard, inputPorts, outputPorts, clock, newPath, model);
    }

    public NodeConfig withBlackboard(Blackboard newBlackboard) {
        return new NodeConfig(newBlackboard, inputPorts, outputPorts, clock, path, model);
    }
}
