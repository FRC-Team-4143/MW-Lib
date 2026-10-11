package com.marswars.auto;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * An auto's XML text as an AdvantageKit input. On the robot and in sim the text read from the file
 * is logged; in log replay the logged text replaces it, so replay rebuilds the tree that actually
 * ran.
 */
final class LoggedXml implements LoggableInputs {
    private final String key_;
    private String xml_ = null;

    LoggedXml(String key) {
        key_ = key;
    }

    /** Logs {@code xml} and returns the text to use: {@code xml}, or the logged text in replay. */
    String process(String xml) {
        xml_ = xml;
        Logger.processInputs(key_, this);
        return xml_;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("XmlInput", xml_ == null ? "" : xml_);
    }

    @Override
    public void fromLog(LogTable table) {
        String logged = table.get("XmlInput", "");
        xml_ = logged.isEmpty() ? null : logged;
    }
}
