package com.marswars.auto;

import choreo.trajectory.DifferentialSample;
import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * A Choreo trajectory (alliance-flipped if needed) plus its event markers, for either a swerve or a
 * differential drivetrain.
 *
 * <p>Swerve users keep using {@link #getTrajectory()}, which returns {@code Trajectory<SwerveSample>}
 * exactly as before. Differential users call {@link #getDifferentialTrajectory()}, or
 * {@link #getTrajectory(Class)} for any sample type. Drivetrain-agnostic data (poses, total time,
 * initial/final pose, events) is available without knowing the sample type.
 */
public class ChoreoTrajectory {

    private final Map<String, Double> event_timestamp_map_;
    private final Map<String, Pose2d> event_pose_map_;

    private final Trajectory<?> trajectory_;
    /** Sample class of this trajectory, or null if it has no samples. */
    private final Class<?> sample_type_;

    /**
     * Swerve constructor (unchanged signature).
     *
     * @param trajectory a swerve trajectory
     * @param is_red_alliance true to flip the trajectory for the red alliance
     */
    public ChoreoTrajectory(Trajectory<SwerveSample> trajectory, boolean is_red_alliance) {
        this(trajectory, is_red_alliance, Void.class);
    }

    private <T extends TrajectorySample<T>> ChoreoTrajectory(
            Trajectory<T> trajectory, boolean is_red_alliance, Class<Void> unused) {
        // flip the trajectory for red alliance if needed
        trajectory_ = is_red_alliance ? trajectory.flipped() : trajectory;
        sample_type_ = trajectory.samples().isEmpty() ? null : trajectory.samples().get(0).getClass();

        // Populate event timestamp and pose maps
        event_timestamp_map_ = new HashMap<>();
        event_pose_map_ = new HashMap<>();
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

    /**
     * Wraps a trajectory of any sample type (swerve or differential).
     *
     * @param trajectory the loaded Choreo trajectory
     * @param is_red_alliance true to flip the trajectory for the red alliance
     * @return the wrapped trajectory
     */
    public static <T extends TrajectorySample<T>> ChoreoTrajectory of(
            Trajectory<T> trajectory, boolean is_red_alliance) {
        return new ChoreoTrajectory(trajectory, is_red_alliance, Void.class);
    }

    /**
     * Wraps a trajectory returned by Choreo without knowing its sample type.
     *
     * @param trajectory the loaded Choreo trajectory
     * @param is_red_alliance true to flip the trajectory for the red alliance
     * @return the wrapped trajectory
     */
    @SuppressWarnings({"unchecked", "rawtypes"})
    static ChoreoTrajectory ofUnknown(Trajectory<?> trajectory, boolean is_red_alliance) {
        return of((Trajectory) trajectory, is_red_alliance);
    }

    /**
     * Gets the trajectory as a swerve trajectory.
     *
     * @return the swerve trajectory
     * @throws IllegalStateException if this is not a swerve trajectory
     */
    @SuppressWarnings("unchecked")
    public Trajectory<SwerveSample> getTrajectory() {
        return getTrajectory(SwerveSample.class);
    }

    /**
     * Gets the trajectory as a differential trajectory.
     *
     * @return the differential trajectory
     * @throws IllegalStateException if this is not a differential trajectory
     */
    public Trajectory<DifferentialSample> getDifferentialTrajectory() {
        return getTrajectory(DifferentialSample.class);
    }

    /**
     * Gets the trajectory with a specific sample type.
     *
     * @param sample_class the expected sample class, e.g. {@code DifferentialSample.class}
     * @return the typed trajectory
     * @throws IllegalStateException if the trajectory's samples are not of that type
     */
    @SuppressWarnings("unchecked")
    public <T extends TrajectorySample<T>> Trajectory<T> getTrajectory(Class<T> sample_class) {
        if (sample_type_ != null && !sample_class.isAssignableFrom(sample_type_)) {
            throw new IllegalStateException("Trajectory " + trajectory_.name() + " has "
                    + sample_type_.getSimpleName() + " samples, not " + sample_class.getSimpleName());
        }
        return (Trajectory<T>) trajectory_;
    }

    /** @return the sample class (SwerveSample, DifferentialSample), or null if the trajectory is empty */
    public Class<?> getSampleType() {
        return sample_type_;
    }

    /** @return true if this trajectory holds {@link DifferentialSample}s */
    public boolean isDifferential() {
        return sample_type_ == DifferentialSample.class;
    }

    /** @return true if this trajectory holds {@link SwerveSample}s */
    public boolean isSwerve() {
        return sample_type_ == SwerveSample.class;
    }

    /** @return the poses of every sample (already alliance-flipped), for visualization */
    public Pose2d[] getPoses() {
        return trajectory_.getPoses();
    }

    /** @return the total time of the trajectory in seconds */
    public double getTotalTime() {
        return trajectory_.getTotalTime();
    }

    /** @return the (already flipped) starting pose, if the trajectory has samples */
    public Optional<Pose2d> getInitialPose() {
        return trajectory_.getInitialPose(false);
    }

    /** @return the (already flipped) final pose, if the trajectory has samples */
    public Optional<Pose2d> getFinalPose() {
        return trajectory_.getFinalPose(false);
    }

    /**
     * Pose at a time along the (already flipped) trajectory.
     *
     * @param timestamp seconds since the start of the trajectory
     * @return the interpolated pose, or empty if the trajectory has no samples
     */
    public Optional<Pose2d> samplePoseAt(double timestamp) {
        return trajectory_.sampleAt(timestamp, false).map(TrajectorySample::getPose);
    }

    public Map<String, Double> getEventTimestampMap() {
        return event_timestamp_map_;
    }

    public Map<String, Pose2d> getEventPoseMap() {
        return event_pose_map_;
    }

}
