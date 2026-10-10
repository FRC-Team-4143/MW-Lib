package com.marswars.bt.core;

/** Result of ticking a {@link TreeNode}. Mirrors BehaviorTree.CPP v4 {@code NodeStatus}. */
public enum NodeStatus {
    /** Not ticked yet, or reset after completing. */
    IDLE('I'),
    /** Ticked and still working; will be ticked again. */
    RUNNING('R'),
    SUCCESS('S'),
    FAILURE('F'),
    /** Skipped by a parent/precondition. Reported to the parent, but the node stays IDLE. */
    SKIPPED('K');

    private final char code_;

    NodeStatus(char code) {
        code_ = code;
    }

    /** One-character code used in compact per-tree status strings. */
    public char code() {
        return code_;
    }

    /** True for SUCCESS or FAILURE. */
    public boolean isCompleted() {
        return this == SUCCESS || this == FAILURE;
    }

    /** Inverse of {@link #code()}. */
    public static NodeStatus fromCode(char code) {
        for (NodeStatus s : values()) {
            if (s.code_ == code) {
                return s;
            }
        }
        throw new IllegalArgumentException("Unknown NodeStatus code '" + code + "'");
    }
}
