package com.marswars.subsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.bt.RobotStateNodes;
import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.NodeStatus;
import edu.wpi.first.hal.HAL;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class MwSuperStateMachineTest {
    enum RobotStates {
        IDLE,
        DRIVE,
        AIM,
        SHOOT,
        SCORE
    }

    enum ShooterStates {
        IDLE,
        SHOOT
    }

    static class ShooterConstants extends MwConstants {}

    /** Records the state it is in each time it updates. */
    static class ShooterSubsystem extends MwSubsystem<ShooterStates, ShooterConstants> {
        final List<ShooterStates> updates = new ArrayList<>();

        ShooterSubsystem() {
            super(ShooterStates.IDLE, new ShooterConstants());
        }

        @Override
        public void updateLogic(double timestamp) {
            updates.add(getSystemState());
        }

        @Override
        public void reset() {
            system_state_ = ShooterStates.IDLE;
        }

        @Override
        public List<SubsystemIoBase> getIos() {
            return List.of();
        }
    }

    /** Robot-level constants, the way a robot project writes them. */
    static class TestRobotConstants extends MwConstants {
        final double SHOOT_SCALE = 0.25;
    }

    static class TestContainer extends MwSuperStateMachine<RobotStates, TestRobotConstants> {
        final ShooterSubsystem shooter = new ShooterSubsystem();
        int resets = 0;

        TestContainer() {
            super(RobotStates.IDLE, new TestRobotConstants(), new Object(), List.of(), false);
            registerSubsystem(shooter);
            reset();
        }

        @Override
        protected void updateLogic(double timestamp) {
            shooter.setWantedState(
                    getSystemState() == RobotStates.SHOOT ? ShooterStates.SHOOT : ShooterStates.IDLE);
        }

        @Override
        public void reset() {
            resets++;
            super.reset();
        }
    }

    TestContainer robot;

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        robot = new TestContainer();
    }

    @Test
    void constantsAreAvailableLikeASubsystem() {
        TestContainer robot = new TestContainer();
        assertEquals(0.25, robot.CONSTANTS.SHOOT_SCALE);
        assertEquals("testrobot", robot.CONSTANTS.getSystemName());
    }

    @Test
    void startsInDefaultState() {
        assertEquals(RobotStates.IDLE, robot.getSystemState());
        assertEquals(RobotStates.IDLE, robot.getWantedState());
    }

    @Test
    void subsystemSeesRequestInSameLoop() {
        robot.setWantedState(RobotStates.SHOOT);
        robot.doControlLoop();

        assertEquals(RobotStates.SHOOT, robot.getSystemState());
        assertEquals(List.of(ShooterStates.SHOOT), robot.shooter.updates);
    }

    @Test
    void wantedStateIsSticky() {
        robot.setWantedState(RobotStates.SHOOT);
        robot.doControlLoop();
        robot.doControlLoop();

        assertEquals(List.of(ShooterStates.SHOOT, ShooterStates.SHOOT), robot.shooter.updates);
    }

    /** A scoring sequence: wanting SCORE walks AIM, then SHOOT, then lands in SCORE. */
    static class SequenceContainer extends TestContainer {
        @Override
        protected void handleStateTransition(RobotStates wanted) {
            if (wanted != RobotStates.SCORE) {
                system_state_ = wanted;
                return;
            }
            switch (system_state_) {
                case AIM -> system_state_ = RobotStates.SHOOT;
                case SHOOT -> system_state_ = RobotStates.SCORE;
                case SCORE -> {}
                default -> system_state_ = RobotStates.AIM;
            }
        }
    }

    @Test
    void sequenceWalksItsStepStates() {
        SequenceContainer seq = new SequenceContainer();
        seq.setWantedState(RobotStates.SCORE);

        seq.doControlLoop();
        assertEquals(RobotStates.AIM, seq.getSystemState());
        seq.doControlLoop();
        assertEquals(RobotStates.SHOOT, seq.getSystemState());
        seq.doControlLoop();
        assertEquals(RobotStates.SCORE, seq.getSystemState());
        assertEquals(RobotStates.SCORE, seq.getWantedState());
    }

    @Test
    void customTransitionCanHoldAState() {
        TestContainer guarded =
                new TestContainer() {
                    @Override
                    protected void handleStateTransition(RobotStates wanted) {
                        // only start shooting from AIM
                        if (wanted == RobotStates.SHOOT && system_state_ != RobotStates.AIM) {
                            system_state_ = RobotStates.AIM;
                        } else {
                            system_state_ = wanted;
                        }
                    }
                };
        guarded.setWantedState(RobotStates.SHOOT);
        guarded.doControlLoop();
        assertEquals(RobotStates.AIM, guarded.getSystemState());
        assertFalse(guarded.isIn(RobotStates.SHOOT));

        guarded.doControlLoop();
        assertTrue(guarded.isIn(RobotStates.SHOOT));
    }

    @Test
    void resetReturnsToDefaultAndResetsSubsystems() {
        robot.setWantedState(RobotStates.SHOOT);
        robot.doControlLoop();
        robot.reset();

        assertEquals(RobotStates.IDLE, robot.getSystemState());
        assertEquals(RobotStates.IDLE, robot.getWantedState());
        assertEquals(ShooterStates.IDLE, robot.shooter.getSystemState());
    }

    @Test
    void behaviorTreeNodesRequestAndCheckRobotStates() {
        BehaviorTreeFactory factory = new BehaviorTreeFactory(() -> 0.0);
        RobotStateNodes.register(factory, () -> robot, RobotStates.class);
        BehaviorTree tree =
                factory.createTreeFromText(
                        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\"><Sequence>"
                                + "<SetWantedState state=\"SHOOT\"/>"
                                + "<IsSystemState state=\"SHOOT\"/>"
                                + "</Sequence></BehaviorTree></root>");

        // the request is instant; the robot isn't in the state until its next loop
        assertEquals(NodeStatus.FAILURE, tree.tickOnce());
        assertEquals(RobotStates.SHOOT, robot.getWantedState());

        robot.doControlLoop();
        assertEquals(NodeStatus.SUCCESS, tree.tickOnce());
    }

    @Test
    void setWantedStateSucceedsAtOnceAndATreeWaitsWithIsSystemState() {
        SequenceContainer seq = new SequenceContainer();
        BehaviorTreeFactory factory = new BehaviorTreeFactory(() -> 0.0);
        RobotStateNodes.register(factory, () -> seq, RobotStates.class);
        BehaviorTree tree =
                factory.createTreeFromText(
                        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\"><Sequence>"
                                + "<SetWantedState state=\"SCORE\"/>"
                                + "<RetryUntilSuccessful num_attempts=\"-1\">"
                                + "<IsSystemState state=\"SCORE\"/></RetryUntilSuccessful>"
                                + "</Sequence></BehaviorTree></root>");

        assertEquals(NodeStatus.RUNNING, tree.tickOnce());
        assertEquals(RobotStates.SCORE, seq.getWantedState());
        seq.doControlLoop(); // AIM
        assertEquals(NodeStatus.RUNNING, tree.tickOnce());
        seq.doControlLoop(); // SHOOT
        assertEquals(NodeStatus.RUNNING, tree.tickOnce());
        seq.doControlLoop(); // SCORE
        assertEquals(NodeStatus.SUCCESS, tree.tickOnce());
    }

    @Test
    void haltingATreeLeavesTheRequestedState() {
        BehaviorTreeFactory factory = new BehaviorTreeFactory(() -> 0.0);
        RobotStateNodes.register(factory, () -> robot, RobotStates.class);
        BehaviorTree tree =
                factory.createTreeFromText(
                        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">"
                                + "<SetWantedState state=\"SHOOT\"/>"
                                + "</BehaviorTree></root>");
        assertEquals(NodeStatus.SUCCESS, tree.tickOnce());
        tree.haltTree();
        assertEquals(RobotStates.SHOOT, robot.getWantedState(), "a request has nothing to undo");
    }
}
