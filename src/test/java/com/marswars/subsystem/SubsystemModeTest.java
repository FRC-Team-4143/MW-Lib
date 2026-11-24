package com.marswars.subsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.bt.RobotStateNodes;
import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.NodeStatus;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

class SubsystemModeTest {
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

    /** A mechanism stand-in that counts what the manager does to it. */
    static class FakeMech implements SwitchableIo {
        final String name;
        int reads = 0;
        int writes = 0;
        int holds = 0;
        int neutral_calls = 0;
        boolean neutral = false;

        FakeMech(String name) {
            this.name = name;
        }

        @Override
        public String getIoName() {
            return name;
        }

        @Override
        public String getLoggingKey() {
            return "Subsystem/Lift/" + name + "/";
        }

        @Override
        public void hold() {
            holds++;
        }

        @Override
        public void setNeutralOutputs(boolean neutral) {
            neutral_calls++;
            this.neutral = neutral;
        }

        @Override
        public void readInputs(double timestamp) {
            reads++;
        }

        @Override
        public void writeOutputs(double timestamp) {
            writes++;
        }

        @Override
        public void logData() {}
    }

    static class LiftSubsystem extends MwSubsystem<LiftStates, LiftConstants> {
        final FakeMech motor = new FakeMech("Lift");
        final FakeMech clamp = new FakeMech("Clamp");
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
            return List.of(motor, clamp);
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
    }

    static final String ROOT = "/MwLib/Subsystems/Lift/";
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

    /** What a dashboard does, then wait for the NT listener to queue the change. */
    void dashboard(String key, Object value) {
        var entry = NetworkTableInstance.getDefault().getEntry(ROOT + key);
        if (value instanceof Boolean b) {
            entry.setBoolean(b);
        } else {
            entry.setString((String) value);
        }
        settle(robot, 1);
    }

    /** NT listeners run on their own thread; wait until {@code manager} has queued n changes. */
    static void settle(SubsystemManager manager, int n) {
        long end = System.nanoTime() + 2_000_000_000L;
        while (manager.pendingModeChanges() < n && System.nanoTime() < end) {
            Thread.onSpinWait();
        }
        assertTrue(manager.pendingModeChanges() >= n, "the change was never queued");
    }

    static String read(String key) {
        return NetworkTableInstance.getDefault().getEntry(ROOT + key).getString("");
    }

    @Test
    void startsEnabledAndRuns() {
        robot.setWantedState(RobotStates.SCORE);
        robot.doControlLoop();

        assertEquals(SubsystemMode.ENABLED, robot.getMode(robot.lift));
        assertEquals(List.of(LiftStates.HIGH), robot.lift.updates);
        assertEquals("ENABLED", read("ActiveMode"));
    }

    @Test
    void disabledIsParkedNeutralButStillReadAndWritten() {
        dashboard("Mode", "DISABLED");
        robot.doControlLoop();
        robot.doControlLoop();

        assertEquals(SubsystemMode.DISABLED, robot.getMode(robot.lift));
        assertEquals(List.of(), robot.lift.updates, "logic must not run");
        assertTrue(robot.lift.motor.neutral);
        assertTrue(robot.lift.clamp.neutral);
        assertEquals(2, robot.lift.motor.reads);
        assertEquals(2, robot.lift.motor.writes);
        assertEquals("DISABLED", read("ActiveMode"));
        assertFalse(robot.isAvailable(robot.lift));
    }

    @Test
    void switchingCostsNothingPerLoop() {
        dashboard("Mode", "DISABLED");
        for (int i = 0; i < 10; i++) {
            robot.doControlLoop();
        }
        assertEquals(1, robot.lift.motor.neutral_calls, "applied once, on the change");
    }

    @Test
    void reEnablingResetsAndFollowsTheRobotInTheSameLoop() {
        robot.setWantedState(RobotStates.SCORE);
        dashboard("Mode", "DISABLED");
        robot.doControlLoop();

        dashboard("Mode", "ENABLED");
        robot.doControlLoop();

        assertEquals(1, robot.lift.resets);
        assertFalse(robot.lift.motor.neutral);
        assertEquals(List.of(LiftStates.HIGH), robot.lift.updates);
    }

    @Test
    void tuningParksTheLogicAndHoldsMechsOnce() {
        dashboard("Mode", "TUNING");
        robot.doControlLoop();
        robot.doControlLoop();

        assertEquals(SubsystemMode.TUNING, robot.getMode(robot.lift));
        assertEquals(List.of(), robot.lift.updates);
        assertEquals(1, robot.lift.motor.holds);
        assertEquals(1, robot.lift.clamp.holds);
        assertFalse(robot.lift.motor.neutral, "tuning drives the mechs");
        assertEquals(2, robot.lift.motor.writes);
    }

    @Test
    void oneMechCanBeSwitchedOffWhileTheSubsystemRuns() {
        dashboard("Mechs/Clamp/Enabled", false);
        robot.doControlLoop();

        assertTrue(robot.lift.clamp.neutral);
        assertFalse(robot.lift.motor.neutral);
        assertEquals(List.of(LiftStates.STOW), robot.lift.updates);
        assertFalse(robot.isMechEnabled(robot.lift, robot.lift.clamp));

        // disabling then re-enabling the subsystem keeps the clamp off
        dashboard("Mode", "DISABLED");
        robot.doControlLoop();
        dashboard("Mode", "ENABLED");
        robot.doControlLoop();
        assertTrue(robot.lift.clamp.neutral);
        assertFalse(robot.lift.motor.neutral);
    }

    @Test
    void switchingAMechBackOnInTuningHoldsIt() {
        dashboard("Mode", "TUNING");
        robot.doControlLoop();
        dashboard("Mechs/Clamp/Enabled", false);
        robot.doControlLoop();
        dashboard("Mechs/Clamp/Enabled", true);
        robot.doControlLoop();

        assertEquals(2, robot.lift.clamp.holds, "once entering tuning, once switched back on");
        assertFalse(robot.lift.clamp.neutral);
    }

    @Test
    void codeRequestsGoThroughTheSamePath() {
        robot.requestMode(robot.lift, SubsystemMode.DISABLED);
        robot.requestMechEnabled(robot.lift, robot.lift.motor, false);
        settle(robot, 2);
        robot.doControlLoop();

        assertEquals(SubsystemMode.DISABLED, robot.getMode(robot.lift));
        assertEquals("DISABLED", read("Mode"), "dashboards see code requests");
        assertFalse(robot.isMechEnabled(robot.lift, robot.lift.motor));
    }

    @Test
    void unknownModeIsIgnored() {
        dashboard("Mode", "sideways");
        robot.doControlLoop();

        assertEquals(SubsystemMode.ENABLED, robot.getMode(robot.lift));
        assertEquals("ENABLED", read("Mode"), "put back to the real mode");
    }

    @Test
    void tuningIsRefusedWithFmsButDisableWorks() {
        setFms(true);
        dashboard("Mode", "TUNING");
        robot.doControlLoop();
        assertEquals(SubsystemMode.ENABLED, robot.getMode(robot.lift));
        assertEquals("ENABLED", read("Mode"));
        // the robot's write-back queues a no-op change, sometimes applied in that same loop;
        // give it time to arrive and apply it either way
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        robot.doControlLoop();

        dashboard("Mode", "DISABLED");
        robot.doControlLoop();
        assertEquals(SubsystemMode.DISABLED, robot.getMode(robot.lift));
    }

    @Test
    void resetWithFmsTakesSubsystemsOutOfTuning() {
        dashboard("Mode", "TUNING");
        robot.doControlLoop();

        setFms(true);
        robot.reset(); // e.g. autonomousInit
        settle(robot, 1);
        robot.doControlLoop();

        assertEquals(SubsystemMode.ENABLED, robot.getMode(robot.lift));
    }

    @Test
    void everyBootStartsEnabled() {
        dashboard("Mode", "DISABLED");
        robot.doControlLoop();

        Robot rebooted = new Robot();
        rebooted.doControlLoop();

        assertEquals(SubsystemMode.ENABLED, rebooted.getMode(rebooted.lift));
        assertEquals("ENABLED", read("Mode"));
    }

    @Test
    void triggerFollowsTheMode() {
        var disabled = robot.inMode(robot.lift, SubsystemMode.DISABLED);
        assertFalse(disabled.getAsBoolean());
        dashboard("Mode", "DISABLED");
        robot.doControlLoop();
        assertTrue(disabled.getAsBoolean());
    }

    @Test
    void autoNodeChecksAvailability() {
        BehaviorTreeFactory factory = new BehaviorTreeFactory(() -> 0.0);
        RobotStateNodes.register(factory, () -> robot, RobotStates.class);
        BehaviorTree tree =
                factory.createTreeFromText(
                        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">"
                                + "<IsSubsystemAvailable subsystem=\"Lift\"/>"
                                + "</BehaviorTree></root>");
        assertEquals(NodeStatus.SUCCESS, tree.tickOnce());

        dashboard("Mode", "TUNING");
        robot.doControlLoop();
        assertEquals(NodeStatus.FAILURE, tree.tickOnce());
    }
}
