package com.marswars.auto;

import edu.wpi.first.wpilibj.DataLogManager;
import java.util.Objects;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.Consumer;

/**
 * Prepares one auto at a time on a background thread ({@link AutoRoutine#prepare}) and runs the
 * step it returns on the robot thread, so reading files and loading paths never stretches a robot
 * loop. Remembers which auto and alliance it prepared last.
 */
final class AutoPreparer {
    /** Where failures go; replaceable for tests. */
    static Consumer<String> message_log_ = DataLogManager::log;

    private record Job(AutoRoutine auto, boolean red, Future<Runnable> result) {}

    private final ExecutorService executor_;
    private final Consumer<AutoRoutine> on_prepared_;
    private Job job_ = null;
    private AutoRoutine prepared_auto_ = null;
    private boolean prepared_red_ = false;

    /**
     * @param executor where {@link AutoRoutine#prepare} runs
     * @param on_prepared called on the robot thread after each auto is put in use
     */
    AutoPreparer(ExecutorService executor, Consumer<AutoRoutine> on_prepared) {
        executor_ = Objects.requireNonNull(executor, "executor");
        on_prepared_ = Objects.requireNonNull(on_prepared, "on_prepared");
    }

    /** One daemon thread named {@code auto-loader}. */
    static ExecutorService backgroundThread() {
        return Executors.newSingleThreadExecutor(
                r -> {
                    Thread t = new Thread(r, "auto-loader");
                    t.setDaemon(true);
                    return t;
                });
    }

    /** True while a preparation is in flight. */
    boolean isBusy() {
        return job_ != null;
    }

    /** True when {@code auto} was the last one prepared, for this alliance. */
    boolean isPrepared(AutoRoutine auto, boolean red) {
        return prepared_auto_ == auto && prepared_red_ == red;
    }

    /** Starts preparing {@code auto} in the background. */
    void submit(AutoRoutine auto, boolean red) {
        job_ = new Job(auto, red, executor_.submit(() -> auto.prepare(red)));
    }

    /** Every robot loop: puts a finished preparation in use. */
    void poll() {
        if (job_ != null && job_.result().isDone()) {
            finishJob();
        }
    }

    /** Waits for the preparation in flight, if any, and puts it in use (tests only). */
    void await() {
        if (job_ != null) {
            finishJob();
        }
    }

    private void finishJob() {
        Job job = job_;
        job_ = null;
        Runnable step = () -> {};
        try {
            step = job.result().get();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (ExecutionException e) {
            message_log_.accept("Preparing auto " + job.auto().getName() + " failed: " + e.getCause());
        }
        use(job.auto(), job.red(), step);
    }

    private void use(AutoRoutine auto, boolean red, Runnable step) {
        step.run();
        prepared_auto_ = auto;
        prepared_red_ = red;
        on_prepared_.accept(auto);
    }
}
