package com.marswars.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * One autonomous routine, as the {@link AutoManager} runs it: chosen on the dashboard, prepared
 * while the robot is disabled, then started in autonomous and stepped every robot loop. It has
 * nothing to do with WPILib commands, so a behavior tree ({@link BehaviorTreeAuto}) runs straight
 * from the robot loop. A command-based auto is just another implementation: {@link Auto} (a {@code
 * SequentialCommandGroup}) is one, and {@link #of(String, Command)} wraps any command; those are
 * run by the {@code CommandScheduler} as before.
 */
public interface AutoRoutine {
    /** Name in the chooser and the logs. */
    String getName();

    /**
     * Gets ready to run on this alliance: reads files, loads and flips paths. {@link AutoManager}
     * calls it on a background thread, so it only computes; it must not change robot state,
     * NetworkTables-backed objects or logs. It returns the step that puts the result in use (swap
     * in the paths, publish errors, log inputs), which {@link AutoManager} runs on the robot
     * thread.
     */
    default Runnable prepare(boolean is_red_alliance) {
        return () -> {};
    }

    /**
     * False when the last {@link #prepare} failed (a broken file, a missing path): the routine has
     * nothing to run. {@link AutoManager} leaves such a routine out of the chooser at boot.
     */
    default boolean isLoaded() {
        return true;
    }

    /** Why the last {@link #prepare} failed, or empty. */
    default String getLoadError() {
        return "";
    }

    /** Where the robot starts (the first path's first pose), or {@link Pose2d#kZero}. */
    default Pose2d getStartPose() {
        return Pose2d.kZero;
    }

    /** Every path pose in order, for the dashboard preview. */
    default Pose2d[] getPath() {
        return new Pose2d[0];
    }

    /** Starts a run (from {@code autonomousInit}). */
    void start();

    /** One step of the run; called every robot loop while {@link #isRunning()}. */
    void periodic();

    /** Ends the run early (disabled, teleop, another auto started). No-op when not running. */
    void stop(boolean interrupted);

    /** True from {@link #start()} until the routine finishes or is stopped. */
    boolean isRunning();

    /** A routine that does nothing (the chooser's default). */
    static AutoRoutine none(String name) {
        return new AutoRoutine() {
            @Override
            public String getName() {
                return name;
            }

            @Override
            public void start() {}

            @Override
            public void periodic() {}

            @Override
            public void stop(boolean interrupted) {}

            @Override
            public boolean isRunning() {
                return false;
            }
        };
    }

    /** Any WPILib command as a routine, run by the {@code CommandScheduler}. */
    static AutoRoutine of(String name, Command command) {
        return new CommandAuto(name, command);
    }
}
