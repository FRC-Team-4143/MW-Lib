package com.marswars.bt.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.marswars.auto.BehaviorTreeAuto;
import com.marswars.auto.ChoreoTrajectory;
import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.Blackboard;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.support.FakeClock;
import com.marswars.bt.xml.BtXmlParser;
import com.marswars.swerve_lib.SwerveFollower;
import com.marswars.swerve_lib.SwerveStates;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** The MW-Lib swerve nodes against a fake drivetrain. */
class SwerveNodesTest {
    static final NodeStatus R = NodeStatus.RUNNING;
    static final NodeStatus S = NodeStatus.SUCCESS;
    static final NodeStatus F = NodeStatus.FAILURE;

    /** Records what the nodes ask for; tests set what it reports. */
    static final class FakeSwerve implements SwerveFollower {
        final List<SwerveStates> states = new ArrayList<>();
        final Set<String> passed_events = new HashSet<>();
        ChoreoTrajectory trajectory = null;
        boolean at_setpoint = false;
        double time_on_path = 0.0;
        boolean stationary = false;
        boolean paused = false;

        @Override
        public void setDesiredChoreoTrajectory(ChoreoTrajectory t) {
            trajectory = t;
        }

        @Override
        public void setWantedState(SwerveStates state) {
            states.add(state);
        }

        @Override
        public boolean isAtChoreoSetpoint() {
            return at_setpoint;
        }

        @Override
        public boolean hasChoreoTimeElapsed(double seconds) {
            return time_on_path >= seconds;
        }

        @Override
        public boolean hasChoreoEventBeenPassed(String event_name) {
            return passed_events.contains(event_name);
        }

        @Override
        public boolean isChassisStationary() {
            return stationary;
        }

        @Override
        public boolean isChoreoTimerPaused() {
            return paused;
        }

        SwerveStates last() {
            return states.get(states.size() - 1);
        }
    }

    final FakeClock clock = new FakeClock();
    final FakeSwerve swerve = new FakeSwerve();
    final List<String> messages = new ArrayList<>();
    final ChoreoTrajectory path = new ChoreoTrajectory(swerveTraj(), false);
    BehaviorTreeFactory factory;

    static Trajectory<SwerveSample> swerveTraj() {
        var samples =
                List.of(
                        new SwerveSample(0, 1, 1, 0, 1, 0, 0, 0, 0, 0, new double[4], new double[4]),
                        new SwerveSample(1, 2, 1, 0, 1, 0, 0, 0, 0, 0, new double[4], new double[4]),
                        new SwerveSample(2, 3, 1, 0, 1, 0, 0, 0, 0, 0, new double[4], new double[4]));
        return new Trajectory<>("P1", samples, List.of(), List.of(new EventMarker(1.0, "mid")));
    }

    @BeforeEach
    void setUp() {
        FollowTrajectoryNode.message_log_ = messages::add;
        factory = new BehaviorTreeFactory(clock);
        SwerveNodes.register(factory, () -> swerve);
    }

    @AfterEach
    void restoreLog() {
        FollowTrajectoryNode.message_log_ = edu.wpi.first.wpilibj.DataLogManager::log;
    }

    /** A tree with the trajectory resolver a BehaviorTreeAuto would put on the root blackboard. */
    BehaviorTree tree(String body) {
        String xml =
                "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\">"
                        + body
                        + "</BehaviorTree></root>";
        Blackboard root = Blackboard.createRoot();
        Function<String, ChoreoTrajectory> resolver = name -> path;
        root.set(BehaviorTreeAuto.TRAJECTORIES_KEY, resolver);
        return factory.createTree(BtXmlParser.parse(xml), root, xml);
    }

    @Test
    void followSucceedsAtTheSetpointAndRequestsTheEndState() {
        BehaviorTree t = tree("<FollowTrajectory trajectory=\"P1\" end_state=\"BRAKE\"/>");
        assertEquals(R, t.tickOnce());
        assertSame(path, swerve.trajectory);
        assertEquals(List.of(SwerveStates.CHOREO_PATH), swerve.states);
        assertEquals(R, t.tickOnce());
        swerve.at_setpoint = true;
        assertEquals(S, t.tickOnce());
        assertEquals(SwerveStates.BRAKE, swerve.last());
    }

    @Test
    void followWaitsForMinTime() {
        BehaviorTree t = tree("<FollowTrajectory trajectory=\"P1\" min_time=\"1.5\"/>");
        t.tickOnce();
        swerve.at_setpoint = true;
        swerve.time_on_path = 1.0;
        assertEquals(R, t.tickOnce());
        swerve.time_on_path = 1.5;
        assertEquals(S, t.tickOnce());
    }

    @Test
    void haltRequestsTheEndState() {
        BehaviorTree t = tree("<FollowTrajectory trajectory=\"P1\" end_state=\"BRAKE\"/>");
        t.tickOnce();
        t.haltTree();
        assertEquals(SwerveStates.BRAKE, swerve.last());
    }

    @Test
    void followGivesUpWhenThePathTimerStaysPaused() {
        BehaviorTree t = tree("<FollowTrajectory trajectory=\"P1\" end_state=\"BRAKE\"/>");
        assertEquals(R, t.tickOnce());
        swerve.paused = true;
        assertEquals(R, t.tickOnce()); // pause starts here
        clock.advance(2.9);
        assertEquals(R, t.tickOnce());
        clock.advance(0.1);
        assertEquals(F, t.tickOnce(), "paused for the default 3 s");
        assertEquals(SwerveStates.BRAKE, swerve.last());
        assertTrue(messages.get(0).contains("gave up on P1"), messages.toString());
    }

    @Test
    void aPauseThatEndsRestartsTheCount() {
        BehaviorTree t = tree("<FollowTrajectory trajectory=\"P1\"/>");
        t.tickOnce();
        swerve.paused = true;
        t.tickOnce();
        clock.advance(2.0);
        assertEquals(R, t.tickOnce());
        swerve.paused = false; // caught up with the path
        assertEquals(R, t.tickOnce());
        swerve.paused = true;
        t.tickOnce();
        clock.advance(2.0);
        assertEquals(R, t.tickOnce(), "two 2 s pauses are not one 3 s pause");
    }

    @Test
    void zeroMaxPausedNeverGivesUp() {
        BehaviorTree t = tree("<FollowTrajectory trajectory=\"P1\" max_paused_sec=\"0\"/>");
        t.tickOnce();
        swerve.paused = true;
        t.tickOnce();
        clock.advance(60.0);
        assertEquals(R, t.tickOnce());
    }

    @Test
    void waitForChoreoEventRunsUntilTheEventPasses() {
        BehaviorTree t = tree("<WaitForChoreoEvent event=\"mid\"/>");
        assertEquals(R, t.tickOnce());
        assertEquals(R, t.tickOnce());
        swerve.passed_events.add("mid");
        assertEquals(S, t.tickOnce());
    }

    @Test
    void setSwerveStateRequestsTheState() {
        assertEquals(S, tree("<SetSwerveState state=\"BRAKE\"/>").tickOnce());
        assertEquals(List.of(SwerveStates.BRAKE), swerve.states);
    }

    @Test
    void conditionsReadTheSwerve() {
        assertEquals(F, tree("<IsAtChoreoSetpoint/>").tickOnce());
        swerve.at_setpoint = true;
        assertEquals(S, tree("<IsAtChoreoSetpoint/>").tickOnce());

        swerve.time_on_path = 0.5;
        assertEquals(F, tree("<HasChoreoTimeElapsed seconds=\"1\"/>").tickOnce());
        swerve.time_on_path = 1.0;
        assertEquals(S, tree("<HasChoreoTimeElapsed seconds=\"1\"/>").tickOnce());

        assertEquals(F, tree("<HasChoreoEventPassed event=\"mid\"/>").tickOnce());
        swerve.passed_events.add("mid");
        assertEquals(S, tree("<HasChoreoEventPassed event=\"mid\"/>").tickOnce());

        assertEquals(F, tree("<IsChassisStationary/>").tickOnce());
        swerve.stationary = true;
        assertEquals(S, tree("<IsChassisStationary/>").tickOnce());
    }
}
