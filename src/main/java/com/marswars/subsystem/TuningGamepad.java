package com.marswars.subsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;
import java.util.List;

/**
 * Drives mechanisms by hand from a spare Xbox controller. It only acts on mechanisms whose
 * subsystem is in {@link SubsystemControl#MANUAL}, and it writes the same entries the dashboard
 * does, so the console shows what the gamepad is doing.
 *
 * <ul>
 *   <li>Bumpers: select the previous / next mechanism (published at {@code
 *       /MwLib/TuningGamepad/Selected})
 *   <li>Left stick Y: duty cycle, scaled to the manual duty-cycle limit; on release the mechanism
 *       holds (position mechanisms where they are, others at zero velocity)
 *   <li>A: start or stop the step test
 *   <li>B: neutral
 * </ul>
 *
 * <pre>{@code
 * // RobotContainer constructor, after registering subsystems
 * new TuningGamepad(this, 3);
 * }</pre>
 *
 * It polls itself on the command scheduler's button loop, so the robot must call {@code
 * CommandScheduler.getInstance().run()} every loop (every command-based robot does).
 */
public class TuningGamepad {
    private static final double DEADBAND = 0.1;

    private final SubsystemManager manager_;
    private final XboxController controller_;
    private final StringPublisher selected_pub_ =
            NetworkTableInstance.getDefault()
                    .getStringTopic("/MwLib/TuningGamepad/Selected")
                    .publish();

    private IoControls selected_ = null;
    private boolean driving_ = false;

    /**
     * @param manager the robot (any {@link SubsystemManager})
     * @param port the controller's driver station port
     */
    public TuningGamepad(SubsystemManager manager, int port) {
        this(manager, new XboxController(port));
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(this::periodic);
    }

    /** For tests: does not bind to the scheduler. */
    TuningGamepad(SubsystemManager manager, XboxController controller) {
        manager_ = manager;
        controller_ = controller;
        selected_pub_.set("");
    }

    /** @return the selected mechanism, or null if none is in MANUAL */
    public IoControls getSelected() {
        return selected_;
    }

    /** Reads the controller and writes the selected mechanism's manual output. */
    void periodic() {
        List<IoControls> candidates = new ArrayList<>();
        for (MwSubsystemBase subsystem : manager_.getSubsystems()) {
            for (IoControls io : manager_.getIoControls(subsystem)) {
                if (io.isManual() && !io.getManualModes().isEmpty()) {
                    candidates.add(io);
                }
            }
        }

        // read edges every loop so presses made while nothing is selected don't linger
        boolean next = controller_.getRightBumperButtonPressed();
        boolean previous = controller_.getLeftBumperButtonPressed();
        boolean step = controller_.getAButtonPressed();
        boolean neutral = controller_.getBButtonPressed();

        if (candidates.isEmpty()) {
            select(null);
            return;
        }
        int index = candidates.indexOf(selected_);
        if (index < 0) {
            index = 0;
        } else if (next) {
            index = (index + 1) % candidates.size();
        } else if (previous) {
            index = (index - 1 + candidates.size()) % candidates.size();
        }
        select(candidates.get(index));

        if (step) {
            selected_.requestStep(!selected_.isStepRequested());
        }
        if (neutral) {
            driving_ = false;
            selected_.requestManual(ManualMode.NEUTRAL, 0.0);
            return;
        }

        double stick = MathUtil.applyDeadband(-controller_.getLeftY(), DEADBAND);
        if (stick != 0.0 && selected_.getManualModes().contains(ManualMode.DUTY_CYCLE)) {
            driving_ = true;
            selected_.requestManual(
                    ManualMode.DUTY_CYCLE, stick * manager_.getManualDutyCycleLimit());
        } else if (driving_) {
            driving_ = false;
            selected_.requestHold();
        }
    }

    private void select(IoControls io) {
        if (io != selected_) {
            if (driving_ && selected_ != null) {
                selected_.requestHold();
            }
            driving_ = false;
            selected_ = io;
            selected_pub_.set(io == null ? "" : io.getSubsystemName() + "/" + io.getIoName());
        }
    }
}
