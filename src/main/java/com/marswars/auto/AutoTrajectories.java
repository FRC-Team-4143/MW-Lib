package com.marswars.auto;

import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * The Choreo trajectories one auto uses, by name, in the order they run. Names are registered
 * first; {@link #load} reads and alliance-flips them all together (safe off the main thread), and
 * {@link #replaceWith} swaps the result in. Shared by {@link Auto} and {@link BehaviorTreeAuto}.
 */
public final class AutoTrajectories {
    /**
     * Parsed (unflipped) trajectories by name. The boot check of every auto parses each {@code
     * .traj} once; later alliance changes only flip. {@link #clearCache()} drops it after new files
     * are copied to the robot.
     */
    private static final Map<String, Trajectory<?>> CACHE =
            new java.util.concurrent.ConcurrentHashMap<>();

    private static final choreo.Choreo.TrajectoryCache CHOREO_CACHE =
            new choreo.Choreo.TrajectoryCache(CACHE);

    /** Reads one {@code deploy/choreo/<name>.traj}; replaceable so unit tests need no files. */
    static Function<String, Optional<Trajectory<?>>> loader_ =
            name -> CHOREO_CACHE.loadTrajectory(name).map(t -> (Trajectory<?>) t);

    /** Forgets every parsed trajectory, so the next load reads the files again. */
    public static void clearCache() {
        CACHE.clear();
    }

    private final Map<String, ChoreoTrajectory> map_ =
            Collections.synchronizedMap(new LinkedHashMap<>());

    /** The live name-to-trajectory map (null until loaded); {@link Auto} exposes it to subclasses. */
    Map<String, ChoreoTrajectory> map() {
        return map_;
    }

    /** Adds a name to load (keeps its loaded trajectory if it is already there). */
    public void register(String name) {
        map_.putIfAbsent(name, null);
    }

    /** Replaces the registered names (nothing loaded yet). */
    public void registerOnly(Collection<String> names) {
        synchronized (map_) {
            map_.clear();
            for (String name : names) {
                map_.put(name, null);
            }
        }
    }

    /** Names in run order. */
    public List<String> names() {
        synchronized (map_) {
            return List.copyOf(map_.keySet());
        }
    }

    /**
     * Reads and flips every named trajectory. Touches no shared state, so it can run on a
     * background thread.
     *
     * @throws IllegalStateException naming every trajectory that has no {@code .traj} file
     */
    public static Map<String, ChoreoTrajectory> load(Collection<String> names, boolean is_red_alliance) {
        Map<String, ChoreoTrajectory> loaded = new LinkedHashMap<>();
        List<String> missing = new ArrayList<>();
        for (String name : names) {
            Optional<Trajectory<?>> traj = loader_.apply(name);
            if (traj.isEmpty()) {
                missing.add(name);
            } else {
                loaded.put(name, new ChoreoTrajectory(traj.get(), is_red_alliance));
            }
        }
        if (!missing.isEmpty()) {
            throw new IllegalStateException("no Choreo trajectory (deploy/choreo/<name>.traj) for " + missing);
        }
        return loaded;
    }

    /** Loads every registered name for this alliance, in place. */
    public void loadAll(boolean is_red_alliance) {
        replaceWith(load(names(), is_red_alliance));
    }

    /** Swaps in a set of loaded trajectories (from {@link #load}). */
    public void replaceWith(Map<String, ChoreoTrajectory> loaded) {
        synchronized (map_) {
            map_.clear();
            map_.putAll(loaded);
        }
    }

    /** Uses a tree's trajectories: the loaded set, or just their names when not loaded yet. */
    void use(List<String> names, Map<String, ChoreoTrajectory> loaded) {
        if (loaded != null) {
            replaceWith(loaded);
        } else {
            registerOnly(names);
        }
    }

    /** A loaded trajectory, looked up when the supplier is called. */
    public Supplier<ChoreoTrajectory> get(String name) {
        return () -> {
            ChoreoTrajectory traj = map_.get(name);
            if (traj == null) {
                throw new IllegalStateException(
                        "Trajectory "
                                + name
                                + " has not been loaded yet. Select the auto (or call"
                                + " cacheTrajetories()) before running it.");
            }
            return traj;
        };
    }

    /** First pose of the first loaded trajectory, or {@link Pose2d#kZero}. */
    public Pose2d startPose() {
        synchronized (map_) {
            if (map_.isEmpty() || map_.values().iterator().next() == null) {
                return Pose2d.kZero;
            }
            return map_.values().iterator().next().getPoses()[0];
        }
    }

    /** Every loaded pose in run order. */
    public Pose2d[] path() {
        synchronized (map_) {
            return map_.values().stream()
                    .filter(t -> t != null)
                    .flatMap(t -> Arrays.stream(t.getPoses()))
                    .toArray(Pose2d[]::new);
        }
    }
}
