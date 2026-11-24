package com.marswars.subsystem;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

class SubsystemControlTest {
    enum RobotStates {
        IDLE,
        SCORE
    }

    enum LiftStates {
        STOW,
        HIGH
    }

    static class LiftConstants extends MwConstants {}

    static class RobotConstants extends MwConstants {}

    /** Records what the manager does to it each loop. */
    static class FakeIo implements SubsystemIoBase {
        int reads = 0;
        int writes = 0;
        final List<Boolean> neutral_at_write = new ArrayList<>();
        private boolean neutral_ = false;

        @Override
        public void readInputs(double timestamp) {
            reads++;
        }

        @Override
        public void setNeutralOutputs(boolean neutral) {
            neutral_ = neutral;
        }

        @Override
        public void writeOutputs(double timestamp) {
            writes++;
            neutral_at_write.add(neutral_);
        }

        @Override
        public void logData() {}
    }

    static class LiftSubsystem extends MwSubsystem<LiftStates, LiftConstants> {
        final FakeIo io = new FakeIo();
        final List<LiftStates> updates = new ArrayList<>();
        int resets = 0;

        LiftSubsystem() {
            super(LiftStates.STOW, new LiftConstants());
        }

        @Override
        public void updateLogic(double timestamp) {
            updates.add(getSystemState());
        }

        @Override
        public void reset() {
            resets++;
            system_state_ = LiftStates.STOW;
        }

        @Override
        public List<SubsystemIoBase> getIos() {
            return List.of(io);
        }
    }

    static class Robot extends MwSuperStateMachine<RobotStates, RobotConstants> {
        final LiftSubsystem lift = new LiftSubsystem();

        Robot() {
            super(RobotStates.IDLE, new RobotConstants(), new Object(), List.of(), false);
            registerSubsystem(lift);
            reset();
        }

        @Override
        protected void updateLogic(double timestamp) {
            lift.setWantedState(
                    getSystemState() == RobotStates.SCORE ? LiftStates.HIGH : LiftStates.STOW);
        }

        void allowOffWithFms(boolean allow) {
            setAllowOffWhenFmsAttached(allow);
        }
    }

    Robot robot;

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        setFms(false);
        robot = new Robot();
        robot.lift.resets = 0;
    }

    @AfterEach
    void tearDown() {
        setFms(false);
    }

    static void setFms(boolean attached) {
        DriverStationSim.setFmsAttached(attached);
        DriverStationSim.notifyNewData();
    }

    /** What a dashboard does. */
    static void dashboardWrite(String key, String value) {
        NetworkTableInstance.getDefault().getEntry("/MwLib/Subsystems/Lift/" + key).setString(value);
    }

    static String dashboardRead(String key) {
        return NetworkTableInstance.getDefault().getEntry("/MwLib/Subsystems/Lift/" + key).getString("");
    }

    @Test
    void startsUnderRobotControl() {
        robot.setWantedState(RobotStates.SCORE);
        robot.doControlLoop();

        assertEquals(SubsystemControl.ROBOT, robot.getControl(robot.lift));
        assertTrue(robot.isAvailable(robot.lift));
        assertEquals(List.of(LiftStates.HIGH), robot.lift.updates);
        assertEquals(List.of(false), robot.lift.io.neutral_at_write);
        assertEquals("ROBOT", dashboardRead("ActiveControl"));
    }

    @Test
    void publishesStateNamesForDashboards() {
        assertArrayEquals(
                new String[] {"STOW", "HIGH"},
                NetworkTableInstance.getDefault()
                        .getEntry("/MwLib/Subsystems/Lift/States")
                        .getStringArray(new String[0]));
    }

    @Test
    void offSkipsLogicButStillReadsAndWritesNeutral() {
        dashboardWrite("Control", "OFF");
        robot.doControlLoop();
        robot.doControlLoop();

        assertEquals(SubsystemControl.OFF, robot.lift.getControl());
        assertEquals(List.of(), robot.lift.updates, "updateLogic must not run");
        assertEquals(2, robot.lift.io.reads, "inputs are still read");
        assertEquals(List.of(true, true), robot.lift.io.neutral_at_write);
        assertEquals("OFF", dashboardRead("ActiveControl"));
        assertFalse(robot.isAvailable(robot.lift));
    }

    @Test
    void requestsAreDroppedWhileOff() {
        dashboardWrite("Control", "OFF");
        robot.setWantedState(RobotStates.SCORE);
        robot.doControlLoop();

        assertEquals(LiftStates.STOW, robot.lift.getWantedState());
    }

    @Test
    void leavingOffResetsAndFollowsTheRobotInTheSameLoop() {
        robot.setWantedState(RobotStates.SCORE);
        dashboardWrite("Control", "OFF");
        robot.doControlLoop();
        assertEquals(0, robot.lift.resets);

        dashboardWrite("Control", "ROBOT");
        robot.doControlLoop();

        assertEquals(1, robot.lift.resets);
        assertEquals(List.of(LiftStates.HIGH), robot.lift.updates);
        assertEquals(List.of(true, false), robot.lift.io.neutral_at_write);
    }

    @Test
    void pinnedHoldsTheStateWhateverTheRobotAsks() {
        dashboardWrite("PinnedState", "HIGH");
        dashboardWrite("Control", "PINNED");
        robot.setWantedState(RobotStates.IDLE); // asks the lift for STOW every loop
        robot.doControlLoop();
        robot.doControlLoop();

        assertEquals(SubsystemControl.PINNED, robot.lift.getControl());
        assertEquals(List.of(LiftStates.HIGH, LiftStates.HIGH), robot.lift.updates);
        assertEquals(0, robot.lift.resets, "pinning from ROBOT keeps the subsystem running");
    }

    @Test
    void unpinningResetsBackToRobotControl() {
        robot.requestPinnedState(robot.lift, LiftStates.HIGH);
        robot.doControlLoop();
        assertEquals("PINNED", dashboardRead("Control"), "code requests show on dashboards");
        assertEquals("HIGH", dashboardRead("PinnedState"));

        robot.requestControl(robot.lift, SubsystemControl.ROBOT);
        robot.doControlLoop();

        assertEquals(1, robot.lift.resets);
        assertEquals(List.of(LiftStates.HIGH, LiftStates.STOW), robot.lift.updates);
    }

    @Test
    void unknownPinnedStateFallsBackToRobot() {
        dashboardWrite("PinnedState", "HIHG");
        dashboardWrite("Control", "PINNED");
        robot.doControlLoop();

        assertEquals(SubsystemControl.ROBOT, robot.lift.getControl());
        assertEquals(List.of(LiftStates.STOW), robot.lift.updates);
    }

    @Test
    void unknownControlFallsBackToRobotAndIsCaseInsensitive() {
        dashboardWrite("Control", "sideways");
        robot.doControlLoop();
        assertEquals(SubsystemControl.ROBOT, robot.lift.getControl());

        dashboardWrite("Control", " off ");
        robot.doControlLoop();
        assertEquals(SubsystemControl.OFF, robot.lift.getControl());
    }

    @Test
    void fmsIgnoresPinnedButHonoursOff() {
        setFms(true);
        dashboardWrite("PinnedState", "HIGH");
        dashboardWrite("Control", "PINNED");
        robot.doControlLoop();
        assertEquals(SubsystemControl.ROBOT, robot.lift.getControl());
        assertEquals(List.of(LiftStates.STOW), robot.lift.updates);

        dashboardWrite("Control", "OFF");
        robot.doControlLoop();
        assertEquals(SubsystemControl.OFF, robot.lift.getControl());
    }

    @Test
    void fmsCanAlsoLockOff() {
        robot.allowOffWithFms(false);
        setFms(true);
        dashboardWrite("Control", "OFF");
        robot.doControlLoop();

        assertEquals(SubsystemControl.ROBOT, robot.lift.getControl());
        assertEquals(List.of(false), robot.lift.io.neutral_at_write);
    }

    @Test
    void newManagerStartsUnderRobotControlWhateverTheDashboardLastWrote() {
        dashboardWrite("Control", "OFF");
        robot.doControlLoop();

        Robot rebooted = new Robot();
        rebooted.doControlLoop();

        assertEquals(SubsystemControl.ROBOT, rebooted.lift.getControl());
        assertEquals("ROBOT", dashboardRead("Control"));
    }

    @Test
    void triggerFollowsControl() {
        Trigger off = robot.inControl(robot.lift, SubsystemControl.OFF);
        assertFalse(off.getAsBoolean());

        robot.requestControl(robot.lift, SubsystemControl.OFF);
        assertFalse(off.getAsBoolean(), "a request applies at the top of the next loop");
        robot.doControlLoop();
        assertTrue(off.getAsBoolean());
    }
}
