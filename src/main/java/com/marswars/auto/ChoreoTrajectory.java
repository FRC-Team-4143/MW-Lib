package com.marswars.auto;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class ChoreoTrajectory {

    private final Map<String, Double> event_timestamp_map_;
    private final Map<String, Pose2d> event_pose_map_;
    
    private Trajectory<SwerveSample> trajectory_;

    public ChoreoTrajectory(Trajectory<SwerveSample> trajectory, boolean is_red_alliance) {
                // flip the trajectory for red alliance if needed
        if(is_red_alliance) {
          trajectory_ = trajectory.flipped();
        } else {
          trajectory_ = trajectory;
        }

        // Populate event timestamp and pose maps
        event_timestamp_map_ = new HashMap<>();
        event_pose_map_ = new HashMap<>();
         // Populate maps with new events
        List<EventMarker> events = trajectory.events();
        for (EventMarker event : events) {
            String eventName = event.event != null ? event.event : "unnamed";
            event_timestamp_map_.put(eventName, event.timestamp);

            // Get the pose at this event's timestamp
            var sample = trajectory.sampleAt(event.timestamp, is_red_alliance);
            if (sample.isPresent()) {
                SwerveSample s = sample.get();
                event_pose_map_.put(eventName, new Pose2d(s.x, s.y, new Rotation2d(s.heading)));
            }
        }
        
    }

    public Trajectory<SwerveSample> getTrajectory() {
        return trajectory_;
    }

    public Pose2d[] getPoses() {
        return trajectory_.samples().stream()
                .map(s -> new Pose2d(s.x, s.y, new Rotation2d(s.heading)))
                .toArray(Pose2d[]::new);
    }

    public Map<String, Double> getEventTimestampMap() {
        return event_timestamp_map_;
    }

    public Map<String, Pose2d> getEventPoseMap() {
        return event_pose_map_;
    }
    
}
