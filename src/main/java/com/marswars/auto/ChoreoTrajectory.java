package com.marswars.auto;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import choreo.trajectory.DifferentialSample;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;

public class ChoreoTrajectory {

    private final Map<String, Double> event_timestamp_map_;
    private final Map<String, Pose2d> event_pose_map_;
    
    private Trajectory<?> trajectory_;

    public ChoreoTrajectory(Trajectory<?> trajectory, boolean is_red_alliance) {
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
                event_pose_map_.put(eventName, sample.get().getPose());
            }
        }
        
    }

    /** @throws IllegalStateException if this is a differential trajectory */
    @SuppressWarnings("unchecked")
    public Trajectory<SwerveSample> getTrajectory() {
        requireSamplesOfType(SwerveSample.class);
        return (Trajectory<SwerveSample>) trajectory_;
    }

    /** @throws IllegalStateException if this is a swerve trajectory */
    @SuppressWarnings("unchecked")
    public Trajectory<DifferentialSample> getDifferentialTrajectory() {
        requireSamplesOfType(DifferentialSample.class);
        return (Trajectory<DifferentialSample>) trajectory_;
    }

    /** Poses of every sample (already alliance-flipped), for either sample type. */
    public Pose2d[] getPoses() {
        return trajectory_.getPoses();
    }

    private void requireSamplesOfType(Class<?> type) {
        var samples = trajectory_.samples();
        if (!samples.isEmpty() && !type.isInstance(samples.get(0))) {
            throw new IllegalStateException("Trajectory " + trajectory_.name() + " has "
                    + samples.get(0).getClass().getSimpleName() + " samples, not " + type.getSimpleName()
                    + "; use the matching getter");
        }
    }

    public Map<String, Double> getEventTimestampMap() {
        return event_timestamp_map_;
    }

    public Map<String, Pose2d> getEventPoseMap() {
        return event_pose_map_;
    }
    
}
