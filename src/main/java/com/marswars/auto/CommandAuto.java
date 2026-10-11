package com.marswars.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Objects;

/**
 * A plain WPILib command as an {@link AutoRoutine}: starting it schedules the command and stopping
 * it cancels it, so it runs (with its requirements) through the {@code CommandScheduler} exactly as
 * a command-based auto always has. See {@link AutoRoutine#of}. Extend {@link Auto} instead when the
 * routine follows Choreo paths, so the dashboard can preview them.
 */
final class CommandAuto implements AutoRoutine {
    private final String name_;
    private final Command command_;

    CommandAuto(String name, Command command) {
        name_ = Objects.requireNonNull(name, "name");
        command_ = Objects.requireNonNull(command, "command");
    }

    @Override
    public String getName() {
        return name_;
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(command_);
    }

    @Override
    public void periodic() {
        // the CommandScheduler runs the command
    }

    @Override
    public void stop(boolean interrupted) {
        if (command_.isScheduled()) {
            command_.cancel();
        }
    }

    @Override
    public boolean isRunning() {
        return command_.isScheduled();
    }
}
