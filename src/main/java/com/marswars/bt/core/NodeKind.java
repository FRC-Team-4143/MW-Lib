package com.marswars.bt.core;

import java.util.Optional;

/** Category of a node; also the element name used by the typed XML form ({@code <Action ID=...>}). */
public enum NodeKind {
    ACTION("Action"),
    CONDITION("Condition"),
    CONTROL("Control"),
    DECORATOR("Decorator"),
    SUBTREE("SubTree");

    private final String xml_tag_;

    NodeKind(String xmlTag) {
        xml_tag_ = xmlTag;
    }

    /** Element name used in BT.CPP XML for the typed node form and in {@code <TreeNodesModel>}. */
    public String xmlTag() {
        return xml_tag_;
    }

    public static Optional<NodeKind> fromXmlTag(String tag) {
        for (NodeKind k : values()) {
            if (k.xml_tag_.equals(tag)) {
                return Optional.of(k);
            }
        }
        return Optional.empty();
    }
}
