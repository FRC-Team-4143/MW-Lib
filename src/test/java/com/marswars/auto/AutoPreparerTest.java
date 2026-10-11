package com.marswars.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.function.Consumer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** The background half runs off this thread; the step it returns runs on the caller's thread. */
class AutoPreparerTest {
    private final ExecutorService executor = AutoPreparer.backgroundThread();
    private final List<String> events = new ArrayList<>();
    private final List<AutoRoutine> shown = new ArrayList<>();
    private final AutoPreparer preparer = new AutoPreparer(executor, shown::add);
    private Consumer<String> saved_log;

    @BeforeEach
    void quietLog() {
        saved_log = AutoPreparer.message_log_;
        AutoPreparer.message_log_ = line -> events.add("log: " + line);
    }

    @AfterEach
    void restore() {
        AutoPreparer.message_log_ = saved_log;
        executor.shutdownNow();
    }

    /** A routine whose prepare blocks until released, then records which thread ran each half. */
    private AutoRoutine routine(String name, CountDownLatch release) {
        return new AutoRoutine() {
            @Override
            public String getName() {
                return name;
            }

            @Override
            public Runnable prepare(boolean red) {
                try {
                    release.await();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                String loader = Thread.currentThread().getName();
                return () ->
                        events.add(
                                name + " loaded on " + loader + ", used on "
                                        + Thread.currentThread().getName());
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

    @Test
    void nothingIsUsedUntilPollSeesTheBackgroundWorkDone() throws Exception {
        CountDownLatch release = new CountDownLatch(1);
        AutoRoutine a = routine("A", release);
        preparer.submit(a, true);
        preparer.poll();
        assertTrue(preparer.isBusy());
        assertFalse(preparer.isPrepared(a, true));
        assertEquals(List.of(), events);

        release.countDown();
        preparer.await();
        assertFalse(preparer.isBusy());
        assertTrue(preparer.isPrepared(a, true));
        assertFalse(preparer.isPrepared(a, false), "prepared for red, not blue");
        String main = Thread.currentThread().getName();
        assertEquals(List.of("A loaded on auto-loader, used on " + main), events);
        assertEquals(List.of(a), shown);
    }

    @Test
    void aFailedPrepareIsLoggedAndStillCountsAsPrepared() {
        AutoRoutine broken = AutoRoutine.of("Broken", Commands.none());
        AutoRoutine throwing =
                new AutoRoutine() {
                    @Override
                    public String getName() {
                        return "Throws";
                    }

                    @Override
                    public Runnable prepare(boolean red) {
                        throw new IllegalStateException("no path");
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
        preparer.submit(throwing, false);
        preparer.await();
        assertTrue(preparer.isPrepared(throwing, false), "not retried on every loop");
        String log = events.get(0);
        assertTrue(log.contains("Preparing auto Throws failed") && log.contains("no path"), log);

        preparer.prepareNow(broken, false);
        assertTrue(preparer.isPrepared(broken, false));
        assertFalse(preparer.isPrepared(throwing, false));
    }
}
