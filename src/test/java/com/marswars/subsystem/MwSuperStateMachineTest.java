package com.marswars.subsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.subsystem.MwSuperStateMachine.StateResult;
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

    static class TestContainer extends MwSuperStateMachine<RobotStates> {
        final ShooterSubsystem shooter = new ShooterSubsystem();
        int resets = 0;

        TestContainer() {
            super(RobotStates.IDLE, new Object(), List.of(), false);
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

    @Test
    void resultIsSuccessOnceInWantedState() {
        robot.setWantedState(RobotStates.SHOOT);
        assertEquals(StateResult.RUNNING, robot.getStateResult());

        robot.doControlLoop();
        assertEquals(StateResult.SUCCESS, robot.getStateResult());
    }

    /** A scoring sequence: wanting SCORE walks AIM, then SHOOT, then lands in SCORE. */
    static class SequenceContainer extends TestContainer {
        boolean piece_missing = false;

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
                default -> {
                    if (piece_missing) {
                        failWantedState();
                    } else {
                        system_state_ = RobotStates.AIM;
                    }
                }
            }
        }
    }

    @Test
    void sequenceReportsRunningUntilItFinishes() {
        SequenceContainer seq = new SequenceContainer();
        seq.setWantedState(RobotStates.SCORE);

        seq.doControlLoop();
        assertEquals(RobotStates.AIM, seq.getSystemState());
        assertEquals(StateResult.RUNNING, seq.getStateResult());
        seq.doControlLoop();
        assertEquals(StateResult.RUNNING, seq.getStateResult());
        seq.doControlLoop();
        assertEquals(RobotStates.SCORE, seq.getSystemState());
        assertEquals(StateResult.SUCCESS, seq.getStateResult());
    }

    @Test
    void failureLastsUntilTheNextRequest() {
        SequenceContainer seq = new SequenceContainer();
        seq.piece_missing = true;
        seq.setWantedState(RobotStates.SCORE);
        seq.doControlLoop();
        assertEquals(StateResult.FAILURE, seq.getStateResult());

        seq.setWantedState(RobotStates.DRIVE);
        assertEquals(StateResult.RUNNING, seq.getStateResult());
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

}
