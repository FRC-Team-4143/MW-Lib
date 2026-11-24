package com.marswars.subsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.bt.RobotStateNodes;
import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.NodeStatus;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class TuningGamepadTest {
    IoControlsTest.Robot robot;
    XboxController controller;
    XboxControllerSim sim;
    TuningGamepad gamepad;

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        DriverStationSim.setFmsAttached(false);
        DriverStationSim.notifyNewData();
        robot = new IoControlsTest.Robot();
        controller = new XboxController(4);
        sim = new XboxControllerSim(controller);
        sim.setLeftY(0.0);
        sim.setRightBumperButton(false);
        sim.setBButton(false);
        sim.setAButton(false);
        DriverStationSim.notifyNewData();
        gamepad = new TuningGamepad(robot, controller);
    }

    /** One robot loop: the gamepad writes, then the robot applies it. */
    void loop() {
        DriverStationSim.notifyNewData();
        gamepad.periodic();
        robot.doControlLoop();
    }

    IoControls pivot() {
        return robot.getIoControls(robot.arm.pivot);
    }

    @Test
    void doesNothingUntilASubsystemIsInManual() {
        sim.setLeftY(-1.0);
        loop();
        assertNull(gamepad.getSelected());
        assertEquals(robot.arm.pivot.manual_applied.size(), 0);
    }

    @Test
    void stickDrivesDutyCycleAndReleaseHolds() {
        robot.requestControl(robot.arm, SubsystemControl.MANUAL);
        loop(); // the robot applies MANUAL after the gamepad has read
        loop();
        assertSame(pivot(), gamepad.getSelected());

        sim.setLeftY(-1.0); // stick up
        loop();
        loop();
        assertEquals(ManualMode.DUTY_CYCLE, pivot().getManualMode());
        assertEquals(0.30, pivot().getManualSetpoint(), 1e-9);

        sim.setLeftY(0.0);
        loop();
        loop();
        assertEquals(ManualMode.POSITION, pivot().getManualMode());
        assertEquals(0.4, pivot().getManualSetpoint(), 1e-9, "holds where it is");
    }

    @Test
    void bumpersCycleMechanismsAndPublishTheSelection() {
        robot.requestControl(robot.arm, SubsystemControl.MANUAL);
        loop(); // the robot applies MANUAL after the gamepad has read
        loop();
        assertSame(pivot(), gamepad.getSelected());

        sim.setRightBumperButton(true);
        loop();
        sim.setRightBumperButton(false);
        loop();
        // order is registration order, with a mech's child IOs right after it
        assertSame(robot.getIoControls(robot.arm.child), gamepad.getSelected());
        assertEquals(
                "Arm/Encoder",
                NetworkTableInstance.getDefault()
                        .getEntry("/MwLib/TuningGamepad/Selected")
                        .getString(""));
    }

    @Test
    void bGoesNeutral() {
        robot.requestControl(robot.arm, SubsystemControl.MANUAL);
        loop(); // the robot applies MANUAL after the gamepad has read
        loop();
        sim.setBButton(true);
        loop();
        loop();
        assertEquals(ManualMode.NEUTRAL, pivot().getManualMode());
    }

    @Test
    void autoNodeChecksSubsystemAvailability() {
        BehaviorTreeFactory factory = new BehaviorTreeFactory(() -> 0.0);
        RobotStateNodes.register(factory, () -> robot, IoControlsTest.RobotStates.class);
        BehaviorTree tree =
                factory.createTreeFromText(
                        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">"
                                + "<IsSubsystemAvailable subsystem=\"Arm\"/>"
                                + "</BehaviorTree></root>");
        robot.doControlLoop();
        assertEquals(NodeStatus.SUCCESS, tree.tickOnce());

        robot.requestControl(robot.arm, SubsystemControl.OFF);
        robot.doControlLoop();
        assertEquals(NodeStatus.FAILURE, tree.tickOnce());
    }
}
