package com.marswars.bt.action;

import com.marswars.logging.MwLog;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * Source of live-tunable numbers for {@link TunableWaitNode}. Trees are rebuilt often (selection,
 * alliance change, every enable), so a registry hands out one supplier per key instead of
 * registering a new tunable each time.
 */
@FunctionalInterface
public interface TunableRegistry {
    /** Returns a supplier of the current value of {@code key}, registering it on first use. */
    DoubleSupplier register(String key, double defaultValue);

    /** Default registry backed by {@link MwLog#tunable} ({@code /Tuning/<key>}, replay-safe). */
    TunableRegistry MWLOG = new MwLogRegistry();

    /** In-memory registry for tests and tools: values can be changed with {@link #set}. */
    final class InMemory implements TunableRegistry {
        private final Map<String, double[]> values_ = new HashMap<>();

        @Override
        public DoubleSupplier register(String key, double defaultValue) {
            double[] holder = values_.computeIfAbsent(key, k -> new double[] {defaultValue});
            return () -> holder[0];
        }

        public void set(String key, double value) {
            values_.computeIfAbsent(key, k -> new double[1])[0] = value;
        }
    }

    /** {@link MwLog#tunable} registration, once per key. */
    final class MwLogRegistry implements TunableRegistry {
        private final Map<String, double[]> values_ = new HashMap<>();

        @Override
        public synchronized DoubleSupplier register(String key, double defaultValue) {
            double[] holder =
                    values_.computeIfAbsent(
                            key,
                            k -> {
                                double[] h = {defaultValue};
                                MwLog.tunable(k, defaultValue, v -> h[0] = v);
                                return h;
                            });
            return () -> holder[0];
        }
    }
}
