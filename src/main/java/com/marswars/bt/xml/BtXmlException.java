package com.marswars.bt.xml;

import com.marswars.bt.core.BtException;

/** Malformed or unsupported BT.CPP XML; carries the 1-based source line when known. */
public class BtXmlException extends BtException {
    private final int line_;

    public BtXmlException(String message, int line) {
        super(line > 0 ? "line " + line + ": " + message : message);
        line_ = line;
    }

    public BtXmlException(String message, int line, Throwable cause) {
        super(line > 0 ? "line " + line + ": " + message : message, cause);
        line_ = line;
    }

    public int line() {
        return line_;
    }
}
