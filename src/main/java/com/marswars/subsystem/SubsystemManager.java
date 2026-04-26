package com.marswars.subsystem;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

import com.marswars.logging.BatteryLogger;
import com.marswars.logging.GitLogger;
import com.marswars.util.ConstantsLoader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public abstract class SubsystemManager {
    protected ArrayList<MwSubsystemBase> subsystems;
    protected Notifier loopThread;
    protected boolean log_init = false;

    private static StringPublisher robot_name_pub_ =
            NetworkTableInstance.getDefault().getStringTopic("/Metadata/ROBOT_NAME").publish();

    // Subsystem enable/disable state management (thread-safe)
    private final ConcurrentHashMap<String, Boolean> subsystem_enabled_state_ = new ConcurrentHashMap<>();
    private final Map<String, BooleanSubscriber> enable_subscribers_ = new HashMap<>();
    
    // Cache of enabled subsystems for efficient iteration (updated only on changes)
    private volatile List<MwSubsystemBase> enabled_subsystems_;

    public SubsystemManager(Object build_constants) {
        // Initialize the subsystem list
        subsystems = new ArrayList<>();
        enabled_subsystems_ = new ArrayList<>();

        DogLogOptions options =
                new DogLogOptions()
                        .withNtPublish(true)
                        .withCaptureNt(true)
                        .withNtTunables(true)
                        .withCaptureDs(true)
                        .withLogExtras(false)
                        .withLogEntryQueueCapacity(1500);

        // setup all logging
        DogLog.setOptions(options);
        DogLog.setEnabled(true);

        // Log robot metadata
        GitLogger.logGitData(build_constants);
        robot_name_pub_.set(ConstantsLoader.getInstance().getRobotName());
        BatteryLogger.logBatteryData();
    }

    /**
     * Register a subsystem to be looped over in the control loop. 
     * @param system The subsystem to register
     */
    public void registerSubsystem(MwSubsystemBase system) {
        subsystems.add(system);
        
        // Initialize subsystem as enabled by default
        String subsystem_key = system.getSubsystemKey();
        boolean default_enabled = true;
        
        // Automatically disable simulation subsystems when not in simulation mode
        if (subsystem_key.contains("Simulation") && !RobotBase.isSimulation()) {
            default_enabled = false;
            DataLogManager.log(
                "Subsystem " + subsystem_key + " DISABLED (not in simulation mode)"
            );
        }
        
        subsystem_enabled_state_.put(subsystem_key, default_enabled);
        
        // Create a NetworkTables subscriber for live enable/disable control
        NetworkTable subsystem_table = NetworkTableInstance.getDefault()
            .getTable("SubsystemManager")
            .getSubTable(subsystem_key);
        
        BooleanTopic enabled_topic = subsystem_table.getBooleanTopic("Enabled");
        BooleanSubscriber enabled_sub = enabled_topic.subscribe(default_enabled);
        enable_subscribers_.put(subsystem_key, enabled_sub);
        
        // Also publish the initial state
        enabled_topic.publish().set(default_enabled);
        
        // Rebuild the enabled subsystems cache
        rebuildEnabledSubsystemsCache();
    }

    /** Preform the control loop for all subsystems */
    public void doControlLoop() {
        // Update enabled states from NetworkTables (only processes changes)
        updateSubsystemEnabledStates();
        
        // Iterate only over enabled subsystems (no per-iteration HashMap lookups)
        for (MwSubsystemBase subsystem : enabled_subsystems_) {
            try {
                String subsystem_key = subsystem.getSubsystemKey();
                DogLog.time(subsystem_key + "/loop_time");

                List<SubsystemIoBase> ios = subsystem.getIos();

                // Run the subsystem update loop
                double timestamp = Timer.getFPGATimestamp();

                for (SubsystemIoBase io : ios) {
                    io.readInputs(timestamp);
                }

                subsystem.update(timestamp);

                for (SubsystemIoBase io : ios) {
                    io.writeOutputs(timestamp);
                    io.logData();
                }

                DogLog.timeEnd(subsystem_key + "/loop_time");
            } catch (Exception e) {
                DataLogManager.log(
                        " Failed to run update loop for "
                                + subsystem.getClass().getCanonicalName());
                e.printStackTrace();
            }
        }
        // Log battery data every loop
        BatteryLogger.logBatteryData();
    }

    /**
     * If subsystems all need to be reset before a robot mode change, call this function to cleanly
     * handle resetting them together. If only one subsystem needs to be reset, that can be accessed
     * through the getInstance method.
     */
    public void reset() {
        for (MwSubsystemBase subsystem : subsystems) {
            subsystem.reset();
        }
    }

    /**
     * Updates the enabled state for all subsystems by reading from NetworkTables.
     * This is called at the beginning of each control loop to handle live enable/disable.
     * Only processes changes (not polled every loop).
     */
    private void updateSubsystemEnabledStates() {
        boolean cache_needs_rebuild = false;
        
        for (Map.Entry<String, BooleanSubscriber> entry : enable_subscribers_.entrySet()) {
            String subsystem_key = entry.getKey();
            BooleanSubscriber subscriber = entry.getValue();
            
            // Read only changed values from the queue (efficient, non-polling)
            boolean[] changes = subscriber.readQueueValues();
            
            // If there were any changes, use the most recent value
            if (changes.length > 0) {
                boolean enabled = changes[changes.length - 1];
                subsystem_enabled_state_.put(subsystem_key, enabled);
                cache_needs_rebuild = true;
                
                // Log to DataLog and DogLog only when state changes
                DataLogManager.log(
                    "Subsystem " + subsystem_key + " " + (enabled ? "ENABLED" : "DISABLED")
                );
                DogLog.log(subsystem_key + "/Enabled", enabled);
            }
        }
        
        // Only rebuild the cache if something changed
        if (cache_needs_rebuild) {
            rebuildEnabledSubsystemsCache();
        }
    }

    /**
     * Rebuilds the cached list of enabled subsystems.
     * This is only called when a subsystem's enabled state changes, not every loop.
     */
    private void rebuildEnabledSubsystemsCache() {
        ArrayList<MwSubsystemBase> new_enabled_list = new ArrayList<>();
        
        for (MwSubsystemBase subsystem : subsystems) {
            String subsystem_key = subsystem.getSubsystemKey();
            if (subsystem_enabled_state_.getOrDefault(subsystem_key, true)) {
                new_enabled_list.add(subsystem);
            }
        }
        
        // Atomic update using volatile field
        enabled_subsystems_ = new_enabled_list;
    }
}
