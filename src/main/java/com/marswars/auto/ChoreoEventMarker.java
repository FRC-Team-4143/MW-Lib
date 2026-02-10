package com.marswars.auto;

import choreo.trajectory.EventMarker;

/** Wrapper class for tracking Choreo event markers with their passed state. */
public class ChoreoEventMarker {
    private final EventMarker event_;
    private boolean has_been_passed_;

    /**
     * Creates a new event marker tracker.
     *
     * @param event The Choreo EventMarker to track
     */
    public ChoreoEventMarker(EventMarker event) {
        this.event_ = event;
        this.has_been_passed_ = false;
    }

    /**
     * Gets the underlying Choreo EventMarker.
     *
     * @return the EventMarker
     */
    public EventMarker getEvent() {
        return event_;
    }

    /**
     * Gets the timestamp when this event should trigger.
     *
     * @return the timestamp in seconds
     */
    public double getTimestamp() {
        return event_.timestamp;
    }

    /**
     * Gets the name of this event.
     *
     * @return the event name
     */
    public String getName() {
        return event_.event != null ? event_.event : "unnamed";
    }

    /**
     * Checks if this event has already been passed.
     *
     * @return true if the event has been passed, false otherwise
     */
    public boolean hasBeenPassed() {
        return has_been_passed_;
    }

    /** Marks this event as passed. */
    public void markPassed() {
        has_been_passed_ = true;
    }

    /** Resets the passed state of this event. */
    public void reset() {
        has_been_passed_ = false;
    }

    /** {@inheritDoc} */
    @Override
    public String toString() {
        return String.format(
                "Event{name='%s', time=%.2f, passed=%b}",
                getName(), getTimestamp(), has_been_passed_);
    }
}
