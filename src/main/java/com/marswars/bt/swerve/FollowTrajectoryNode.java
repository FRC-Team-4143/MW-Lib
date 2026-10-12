package com.marswars.bt.swerve;

import com.marswars.auto.BehaviorTreeAuto;
import com.marswars.auto.ChoreoTrajectory;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.PortType;
import com.marswars.bt.core.StatefulActionNode;
import com.marswars.swerve_lib.MwSwerveSubsystem;
import com.marswars.swerve_lib.SwerveStates;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Drives a Choreo trajectory: selects it on the swerve, requests {@code path_state}, and succeeds
 * once {@code isAtChoreoSetpoint()} (and at least {@code min_time} seconds of the path have run).
 * On success, failure or halt, requests {@code end_state}. The trajectory comes from the running
 * {@link BehaviorTreeAuto}'s alliance-flipped cache, which pre-loads every {@code trajectory}
 * literal.
 *
 * <p>The swerve pauses the path timer while the robot lags the path (pushed, blocked). If it stays
 * paused for {@code max_paused_sec} seconds the node gives up and returns FAILURE, so a stuck robot
 * doesn't stall the whole auto. Wrap it in {@code ForceSuccess} to carry on with the auto anyway.
 */
public class FollowTrajectoryNode extends StatefulActionNode {
    public static final String ID = "FollowTrajectory";
    public static final String MAX_PAUSED_SEC = "max_paused_sec";

    /** Where the give-up message goes; replaceable for tests. */
    static Consumer<String> message_log_ = DataLogManager::log;

    public static final List<PortInfo> PORTS =
            List.of(
                    PortInfo.trajectory(
                            "trajectory", "Choreo trajectory name (deploy/choreo/<name>.traj)"),
                    PortInfo.enumInput(
                            "path_state",
                            SwerveStates.class,
                            SwerveStates.CHOREO_PATH,
                            "Swerve state used while following (CHOREO_PATH or"
                                    + " CHOREO_PATH_ROTATION_LOCK)"),
                    PortInfo.enumInput(
                            "end_state",
                            SwerveStates.class,
                            SwerveStates.IDLE,
                            "Swerve state requested when the path finishes, gives up or is halted"),
                    PortInfo.input(
                            "min_time",
                            PortType.DOUBLE,
                            "0",
                            "Minimum seconds on the path before it may succeed"),
                    PortInfo.input(
                            MAX_PAUSED_SEC,
                            PortType.DOUBLE,
                            "3",
                            "Fail once the path timer has been paused this long (the robot is stuck"
                                    + " behind the path); 0 never gives up"));

    private final Supplier<? extends MwSwerveSubsystem<?>> swerve_;
    private String trajectory_name_ = "";
    private SwerveStates end_state_ = SwerveStates.IDLE;
    private double min_time_ = 0.0;
    private double max_paused_ = 0.0;
    private double pause_start_ = Double.NaN;

    public FollowTrajectoryNode(
            String name, NodeConfig config, Supplier<? extends MwSwerveSubsystem<?>> swerve) {
        super(name, config);
        swerve_ = Objects.requireNonNull(swerve, "swerve");
    }

    @Override
    protected NodeStatus onStart() {
        trajectory_name_ = getString("trajectory");
        ChoreoTrajectory trajectory = BehaviorTreeAuto.trajectories(this).apply(trajectory_name_);
        end_state_ = getEnum("end_state", SwerveStates.class);
        min_time_ = getDouble("min_time");
        max_paused_ = getDouble(MAX_PAUSED_SEC);
        pause_start_ = Double.NaN;

        MwSwerveSubsystem<?> swerve = swerve_.get();
        swerve.setDesiredChoreoTrajectory(trajectory);
        swerve.setWantedState(getEnum("path_state", SwerveStates.class));
        return NodeStatus.RUNNING;
    }

    @Override
    protected NodeStatus onRunning() {
        MwSwerveSubsystem<?> swerve = swerve_.get();
        if (max_paused_ > 0.0) {
            if (!swerve.isChoreoTimerPaused()) {
                pause_start_ = Double.NaN;
            } else if (Double.isNaN(pause_start_)) {
                pause_start_ = now();
            } else if (now() - pause_start_ >= max_paused_) {
                swerve.setWantedState(end_state_);
                message_log_.accept(
                        "[BehaviorTree] "
                                + getPath()
                                + ": gave up on "
                                + trajectory_name_
                                + ", path timer paused for "
                                + max_paused_
                                + " s");
                return NodeStatus.FAILURE;
            }
        }
        if (swerve.isAtChoreoSetpoint() && swerve.hasChoreoTimeElapsed(min_time_)) {
            swerve.setWantedState(end_state_);
            return NodeStatus.SUCCESS;
        }
        return NodeStatus.RUNNING;
    }

    @Override
    protected void onHalted() {
        swerve_.get().setWantedState(end_state_);
    }
}
