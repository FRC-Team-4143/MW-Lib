package com.marswars.subsystem;

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

    protected String getSystemName() {
        return system_name;
    }
}
