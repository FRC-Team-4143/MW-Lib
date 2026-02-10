package com.marswars.auto;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Tracks and manages Choreo trajectory event markers using a HashMap. This class maintains a map of
 * event names to their passed state and provides triggers that can be used to schedule commands
 * when events are reached.
 */
public class ChoreoEventTracker {
    private final Map<String, Boolean> event_passed_map_;
    private final Map<String, Double> event_timestamp_map_;
    private final Map<String, Pose2d> event_pose_map_;
    private final Map<String, Trigger> event_triggers_;
    private final String log_key_;

    private Trajectory<SwerveSample> current_trajectory_;
    private Supplier<Pose2d> robot_pose_supplier_;
    private boolean flip_for_red_;
    private double current_time_;
    private boolean is_active_;

    /**
     * Creates a new event tracker.
     *
     * @param log_key The base key for logging event information
     * @param robot_pose_supplier Supplier that provides the current robot pose
     */
    public ChoreoEventTracker(String log_key, Supplier<Pose2d> robot_pose_supplier) {
        event_passed_map_ = new HashMap<>();
        event_timestamp_map_ = new HashMap<>();
        event_pose_map_ = new HashMap<>();
        event_triggers_ = new HashMap<>();
        log_key_ = log_key;
        robot_pose_supplier_ = robot_pose_supplier;
        current_time_ = 0.0;
        is_active_ = false;
    }

    /**
     * Sets the events for the current trajectory and resets all tracking state. Creates a HashMap
     * entry for each event, initialized to false (not passed).
     *
     * @param trajectory The trajectory containing the events
     * @param flip_for_red Whether to flip the trajectory for red alliance
     */
    public void setEvents(Trajectory<SwerveSample> trajectory, boolean flip_for_red) {
        current_trajectory_ = trajectory;
        flip_for_red_ = flip_for_red;

        // Clear existing maps
        event_passed_map_.clear();
        event_timestamp_map_.clear();
        event_pose_map_.clear();
        event_triggers_.clear();

        // Populate maps with new events
        List<EventMarker> events = trajectory.events();
        for (EventMarker event : events) {
            String eventName = event.event != null ? event.event : "unnamed";
            event_passed_map_.put(eventName, false);
            event_timestamp_map_.put(eventName, event.timestamp);

            // Get the pose at this event's timestamp
            var sample = trajectory.sampleAt(event.timestamp, flip_for_red);
            if (sample.isPresent()) {
                event_pose_map_.put(eventName, sample.get().getPose());
            }
        }

        current_time_ = 0.0;
        is_active_ = false;

        DogLog.log(log_key_ + "EventCount", event_passed_map_.size());
        DogLog.log(log_key_ + "EventNames", event_passed_map_.keySet().toArray(new String[0]));
    }

    /**
     * Updates the tracker with the current trajectory time. Checks all events and marks them as
     * passed if the current time has reached their timestamp.
     *
     * @param trajectory_time The current time in the trajectory (in seconds)
     */
    public void update(double trajectory_time) {
        if (!is_active_) {
            return;
        }

        current_time_ = trajectory_time;

        // Check each event to see if it should be marked as passed
        for (Map.Entry<String, Double> entry : event_timestamp_map_.entrySet()) {
            String event_name = entry.getKey();
            double event_timestamp = entry.getValue();

            // If not already passed and we've reached the timestamp, mark it as passed
            if (!event_passed_map_.get(event_name) && current_time_ >= event_timestamp) {
                event_passed_map_.put(event_name, true);
                DogLog.log(log_key_ + "PassedEvent", event_name);
                DogLog.log(log_key_ + "PassedEventTime", current_time_);
            }
        }
    }

    /** Starts tracking events. This should be called when trajectory following begins. */
    public void start() {
        is_active_ = true;
        current_time_ = 0.0;

        // Reset all events to not passed
        for (String event_name : event_passed_map_.keySet()) {
            event_passed_map_.put(event_name, false);
        }

        DogLog.log(log_key_ + "EventTracking", "Started");
    }

    /** Stops tracking events. This should be called when trajectory following ends. */
    public void stop() {
        is_active_ = false;
        DogLog.log(log_key_ + "EventTracking", "Stopped");
    }

    /**
     * Gets a trigger for the specified event. The trigger will be true from the moment the event
     * timestamp is passed until the trajectory ends or a new trajectory is loaded.
     *
     * <p>Use with .onTrue() for one-time actions or .whileTrue() for continuous actions.
     *
     * @param event_name The name of the event to trigger on
     * @return A Trigger that reads from the event HashMap
     */
    public Trigger getTimeTrigger(String event_name) {
        // Cache triggers to avoid creating multiple triggers for the same event
        return event_triggers_.computeIfAbsent(
                event_name, name -> new Trigger(() -> hasEventBeenPassed(name)));
    }

    /**
     * Gets a trigger for the specified event that checks if the robot is at the event's pose. This
     * is a pose-only check (does not require time condition). The trigger will be true when the
     * robot is within tolerances of the event's pose.
     *
     * @param event_name The name of the event to trigger on
     * @param translation_tol The translation distance tolerance in meters
     * @param rotation_tol The rotation tolerance in radians
     * @return A Trigger that checks pose only
     */
    public Trigger getPoseTrigger(
            String event_name, double translation_tol, double rotation_tol) {
        return new Trigger(
                () -> {
                    if (!is_active_) {
                        return false;
                    }

                    // Check pose only (no time requirement)
                    Pose2d event_pose = event_pose_map_.get(event_name);
                    if (event_pose == null) {
                        return false;
                    }

                    Pose2d robot_pose = robot_pose_supplier_.get();

                    // Check translation
                    double translation_distance =
                            robot_pose.getTranslation().getDistance(event_pose.getTranslation());
                    if (translation_distance > translation_tol) {
                        return false;
                    }

                    // Check rotation
                    double rotation_error =
                            Math.abs(
                                    robot_pose
                                            .getRotation()
                                            .minus(event_pose.getRotation())
                                            .getRadians());

                    return rotation_error <= rotation_tol;
                });
    }

    /**
     * Gets the pose associated with an event (the pose at the event's timestamp).
     *
     * @param event_name The name of the event
     * @return The pose at the event, or null if not found
     */
    public Pose2d getEventPose(String event_name) {
        return event_pose_map_.get(event_name);
    }

    /**
     * Checks if a specific event has been passed.
     *
     * @param event_name The name of the event to check
     * @return true if the event has been passed, false otherwise
     */
    public boolean hasEventBeenPassed(String event_name) {
        if (!is_active_) {
            return false;
        }
        return event_passed_map_.getOrDefault(event_name, false);
    }

    /**
     * Gets the timestamp of a specific event.
     *
     * @param event_name The name of the event
     * @return The timestamp of the event, or -1 if not found
     */
    public double getEventTimestamp(String event_name) {
        return event_timestamp_map_.getOrDefault(event_name, -1.0);
    }

    /**
     * Gets a copy of the event passed map.
     *
     * @return A map of event names to their passed state
     */
    public Map<String, Boolean> getEventPassedMap() {
        return new HashMap<>(event_passed_map_);
    }
}
