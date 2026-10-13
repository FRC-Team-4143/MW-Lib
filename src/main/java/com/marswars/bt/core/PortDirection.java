package com.marswars.bt.core;

/** Direction of a node port, with the BT.CPP {@code <TreeNodesModel>} element name. */
public enum PortDirection {
    INPUT("input_port"),
    OUTPUT("output_port"),
    INOUT("inout_port");

    private final String xml_tag_;

    PortDirection(String xmlTag) {
        xml_tag_ = xmlTag;
    }

    public String xmlTag() {
        return xml_tag_;
    }
}
