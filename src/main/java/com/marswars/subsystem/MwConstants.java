package com.marswars.subsystem;

import com.marswars.util.MWPreferences;
import edu.wpi.first.wpilibj.DataLogManager;

public class MwConstants {
    private final String system_name;

    protected MwConstants() {
        // Use some Java magic to pull the class name
        String name = this.getClass().getSimpleName();
        name = name.substring(name.lastIndexOf('.') + 1);
        if (name.endsWith("Constants")) {
            name = name.substring(0, name.length() - "Constants".length());
        }

        system_name = name.toLowerCase();

        DataLogManager.log("Loading constants for " + system_name);
    }

    /**
     * Robot preferences, for values a subsystem stores on the roboRIO itself (a zeroed arm offset,
     * a module offset) rather than in code.
     */
    protected final MWPreferences PREFERENCES = MWPreferences.getInstance();

    protected String getSystemName() {
        return system_name;
    }
}
