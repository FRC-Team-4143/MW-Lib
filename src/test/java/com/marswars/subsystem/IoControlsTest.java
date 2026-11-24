package com.marswars.subsystem;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

class IoControlsTest {
    enum RobotStates {
        IDLE,
        SCORE
    }

    enum ArmStates {
        STOW,
        UP
    }

    static class ArmConstants extends MwConstants {}

    static class RobotConstants extends MwConstants {}

    /** A mechanism-like IO: records what it was told to do. */
    static class FakeMech implements ControllableIo {
        final String name;
        double measured_position = 0.4;
        IoControls controls;
        boolean manager_neutral = false;
        final List<String> manual_applied = new ArrayList<>();
        final List<Boolean> neutral_at_write = new ArrayList<>();
        final List<ControllableIo> children = new ArrayList<>();

        FakeMech(String name) {
            this.name = name;
        }

        @Override
        public String getIoName() {
            return name;
        }

        @Override
        public String getLoggingKey() {
            return "Subsystem/Arm/" + name + "/";
        }

        @Override
        public void attachControls(IoControls controls) {
            this.controls = controls;
            controls.declareSensor("Position");
        }

        @Override
        public List<ControllableIo> getChildIos() {
            return children;
        }

        @Override
        public List<ManualMode> getManualModes() {
            return List.of(
                    ManualMode.NEUTRAL,
                    ManualMode.POSITION,
                    ManualMode.VELOCITY,
                    ManualMode.DUTY_CYCLE);
        }

        @Override
        public void applyManualOutput(ManualMode mode, double setpoint) {
            manual_applied.add(mode + " " + setpoint);
        }

        @Override
        public double getManualSeed(ManualMode mode) {
            return mode == ManualMode.POSITION ? measured_position : 0.0;
        }

        double getPosition() {
            return controls.sensorValue("Position", measured_position);
        }

        @Override
        public void setNeutralOutputs(boolean neutral) {
            manager_neutral = neutral;
        }

        @Override
        public boolean isNeutralOutputs() {
            return manager_neutral || controls.holdsNeutral();
        }

        @Override
        public void readInputs(double timestamp) {}

        @Override
        public void writeOutputs(double timestamp) {
            neutral_at_write.add(isNeutralOutputs());
        }

        @Override
        public void logData() {}
    }

    static class ArmSubsystem extends MwSubsystem<ArmStates, ArmConstants> {
        final FakeMech pivot = new FakeMech("Pivot");
        final FakeMech wrist = new FakeMech("Pivot"); // same name on purpose
        final FakeMech child = new FakeMech("Encoder");
        final List<ArmStates> updates = new ArrayList<>();
        int resets = 0;

        ArmSubsystem() {
            super(ArmStates.STOW, new ArmConstants());
            pivot.children.add(child);
        }

        @Override
        public void updateLogic(double timestamp) {
            updates.add(getSystemState());
        }

        @Override
        public void reset() {
            resets++;
            system_state_ = ArmStates.STOW;
        }

        @Override
        public List<SubsystemIoBase> getIos() {
            return List.of(pivot, wrist);
        }
    }

    static class Robot extends MwSuperStateMachine<RobotStates, RobotConstants> {
        final ArmSubsystem arm = new ArmSubsystem();

        Robot() {
            super(RobotStates.IDLE, new RobotConstants(), new Object(), List.of(), false);
            registerSubsystem(arm);
            reset();
        }

        @Override
        protected void updateLogic(double timestamp) {
            arm.setWantedState(getSystemState() == RobotStates.SCORE ? ArmStates.UP : ArmStates.STOW);
        }

        void dutyLimit(double limit) {
            setManualDutyCycleLimit(limit);
        }

        void tuneOnlyInManual(boolean only) {
            setTuneOnlyInManual(only);
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
        robot.arm.resets = 0;
    }

    @AfterEach
    void tearDown() {
        setFms(false);
        SimHooks.resumeTiming();
    }

    static void setFms(boolean attached) {
        DriverStationSim.setFmsAttached(attached);
        DriverStationSim.notifyNewData();
    }

    static final String IO = "/MwLib/Subsystems/Arm/IO/Pivot/";

    static NetworkTableInstance nt() {
        return NetworkTableInstance.getDefault();
    }

    @Test
    void everyLibraryIoGetsControlsWithUniqueNames() {
        assertNotNull(robot.getIoControls(robot.arm.pivot));
        assertNotNull(robot.getIoControls(robot.arm.wrist));
        assertNotNull(robot.getIoControls(robot.arm.child), "child IOs are attached too");
        assertTrue(nt().getEntry("/MwLib/Subsystems/Arm/IO/Pivot2/Enabled").getBoolean(false));
        assertTrue(nt().getEntry("/MwLib/Subsystems/Arm/IO/Encoder/Enabled").getBoolean(false));
        assertArrayEquals(
                new String[] {"NEUTRAL", "POSITION", "VELOCITY", "DUTY_CYCLE"},
                nt().getEntry(IO + "Manual/Modes").getStringArray(new String[0]));
    }

    @Test
    void switchingOffOneIoHoldsItNeutralWhileTheSubsystemRuns() {
        nt().getEntry(IO + "Enabled").setBoolean(false);
        robot.doControlLoop();

        assertFalse(robot.arm.pivot.controls.isEnabled());
        assertEquals(List.of(true), robot.arm.pivot.neutral_at_write);
        assertEquals(List.of(false), robot.arm.wrist.neutral_at_write, "the other IO keeps going");
        assertEquals(List.of(ArmStates.STOW), robot.arm.updates, "the subsystem keeps running");

        nt().getEntry(IO + "Enabled").setBoolean(true);
        robot.doControlLoop();
        assertEquals(List.of(true, false), robot.arm.pivot.neutral_at_write);
    }

    @Test
    void sensorOverrideReplacesTheReading() {
        nt().getEntry(IO + "Override/Position/Value").setDouble(1.25);
        nt().getEntry(IO + "Override/Position/Active").setBoolean(true);
        robot.doControlLoop();
        assertEquals(1.25, robot.arm.pivot.getPosition());

        nt().getEntry(IO + "Override/Position/Active").setBoolean(false);
        robot.doControlLoop();
        assertEquals(0.4, robot.arm.pivot.getPosition());
    }

    @Test
    void sensorOverridesAreIgnoredWithFms() {
        setFms(true);
        nt().getEntry(IO + "Override/Position/Value").setDouble(1.25);
        nt().getEntry(IO + "Override/Position/Active").setBoolean(true);
        robot.doControlLoop();
        assertEquals(0.4, robot.arm.pivot.getPosition());
    }

    @Test
    void manualSkipsLogicAndHoldsPositionMechsWhereTheyAre() {
        robot.setWantedState(RobotStates.SCORE);
        robot.requestControl(robot.arm, SubsystemControl.MANUAL);
        robot.doControlLoop();

        assertEquals(List.of(), robot.arm.updates, "logic must not run in MANUAL");
        assertEquals(List.of("POSITION 0.4"), robot.arm.pivot.manual_applied);
        assertEquals("POSITION", nt().getEntry(IO + "Manual/Mode").getString(""));
        assertEquals(0.4, nt().getEntry(IO + "Manual/Setpoint").getDouble(0));
        assertEquals(List.of(false), robot.arm.pivot.neutral_at_write);
    }

    @Test
    void manualFollowsTheDashboardAndClampsDutyCycle() {
        robot.requestControl(robot.arm, SubsystemControl.MANUAL);
        robot.doControlLoop();

        nt().getEntry(IO + "Manual/Mode").setString("DUTY_CYCLE");
        nt().getEntry(IO + "Manual/Setpoint").setDouble(0.9);
        robot.doControlLoop();
        assertEquals("DUTY_CYCLE 0.3", robot.arm.pivot.manual_applied.get(1));

        robot.dutyLimit(0.5);
        nt().getEntry(IO + "Manual/Setpoint").setDouble(-0.9);
        robot.doControlLoop();
        assertEquals("DUTY_CYCLE -0.5", robot.arm.pivot.manual_applied.get(2));
    }

    @Test
    void manualNeutralAndUnsupportedModesHoldNeutral() {
        robot.requestControl(robot.arm, SubsystemControl.MANUAL);
        robot.doControlLoop();

        nt().getEntry(IO + "Manual/Mode").setString("NEUTRAL");
        robot.doControlLoop();
        nt().getEntry(IO + "Manual/Mode").setString("CURRENT"); // not in this IO's list
        robot.doControlLoop();

        assertEquals(List.of("POSITION 0.4"), robot.arm.pivot.manual_applied);
        assertEquals(List.of(false, true, true), robot.arm.pivot.neutral_at_write);
        assertEquals(ManualMode.NEUTRAL, robot.arm.pivot.controls.getManualMode());
    }

    @Test
    void stepTestAlternatesBetweenSetpointAndStepTo() {
        SimHooks.pauseTiming();
        robot.requestControl(robot.arm, SubsystemControl.MANUAL);
        robot.doControlLoop();

        nt().getEntry(IO + "Manual/Setpoint").setDouble(0.0);
        nt().getEntry(IO + "Manual/StepTo").setDouble(1.0);
        nt().getEntry(IO + "Manual/StepPeriod").setDouble(1.0);
        nt().getEntry(IO + "Manual/Step").setBoolean(true);

        List<Double> seen = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            robot.doControlLoop();
            seen.add(robot.arm.pivot.controls.getManualSetpoint());
            SimHooks.stepTiming(1.0);
        }
        assertEquals(2, seen.stream().filter(v -> v == 0.0).count());
        assertEquals(2, seen.stream().filter(v -> v == 1.0).count());
        assertTrue(!seen.get(0).equals(seen.get(1)), "it changes every period");
    }

    @Test
    void leavingManualResetsAndHandsBackToTheRobot() {
        robot.setWantedState(RobotStates.SCORE);
        robot.requestControl(robot.arm, SubsystemControl.MANUAL);
        robot.doControlLoop();
        robot.requestControl(robot.arm, SubsystemControl.ROBOT);
        robot.doControlLoop();

        assertEquals(1, robot.arm.resets);
        assertEquals(List.of(ArmStates.UP), robot.arm.updates);
        assertEquals(ManualMode.NEUTRAL, robot.arm.pivot.controls.getManualMode());
    }

    @Test
    void manualIsIgnoredWithFms() {
        setFms(true);
        robot.requestControl(robot.arm, SubsystemControl.MANUAL);
        robot.doControlLoop();

        assertEquals(SubsystemControl.ROBOT, robot.getControl(robot.arm));
        assertEquals(List.of(), robot.arm.pivot.manual_applied);
        assertEquals(List.of(ArmStates.STOW), robot.arm.updates);
    }

    @Test
    void tunablesOnlyChangeInManual() {
        robot.doControlLoop();
        assertFalse(robot.acceptsTunableChange("Subsystem/Arm/Pivot/PositionGains/kP"));
        assertTrue(robot.acceptsTunableChange("Auto/SomethingElse"), "unowned keys stay live");
        assertTrue(
                robot.acceptsTunableChange("Subsystem/Arm/HeadingGains/kP"),
                "subsystem-level gains need the logic running, so they stay live");

        robot.requestControl(robot.arm, SubsystemControl.MANUAL);
        robot.doControlLoop();
        assertTrue(robot.acceptsTunableChange("Subsystem/Arm/Pivot/PositionGains/kP"));

        setFms(true);
        assertFalse(robot.acceptsTunableChange("Subsystem/Arm/Pivot/PositionGains/kP"));
        assertFalse(robot.acceptsTunableChange("Auto/SomethingElse"), "nothing changes in a match");
        setFms(false);

        robot.tuneOnlyInManual(false);
        robot.requestControl(robot.arm, SubsystemControl.ROBOT);
        robot.doControlLoop();
        assertTrue(robot.acceptsTunableChange("Subsystem/Arm/Pivot/PositionGains/kP"));
    }
}
