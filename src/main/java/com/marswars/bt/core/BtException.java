package com.marswars.bt.core;

/** Runtime error raised by the behavior-tree engine (bad tree, bad port, illegal status). */
public class BtException extends RuntimeException {
    public BtException(String message) {
        super(message);
    }

    public BtException(String message, Throwable cause) {
        super(message, cause);
    }
}
