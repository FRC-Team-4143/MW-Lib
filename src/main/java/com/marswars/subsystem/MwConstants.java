package com.marswars.subsystem;

import edu.wpi.first.wpilibj.DataLogManager;
import com.marswars.util.ConstantsLoader;
import com.marswars.util.MWPreferences;

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

  // ConstantsLoader instance for loading configuration values
  protected final ConstantsLoader LOADER = ConstantsLoader.getInstance();

  // MW Preferences loader for loading from and to preferences
  protected final MWPreferences PREFERENCES = MWPreferences.getInstance();

  protected String getSystemName() {
    return system_name;
  }

  protected final double getDoubleConstant(String... path_steps) {
    String[] arr = new String[path_steps.length + 1];
    arr[0] = system_name;
    for (int i = 0; i < path_steps.length; i++) {
      arr[i + 1] = path_steps[i];
    }
    return LOADER.getDoubleValue(arr);
  }

  protected final int getIntConstant(String... path_steps) {
    String[] arr = new String[path_steps.length + 1];
    arr[0] = system_name;
    for (int i = 0; i < path_steps.length; i++) {
      arr[i + 1] = path_steps[i];
    }
    return LOADER.getIntValue(arr);
  }

  protected final String getStringConstant(String... path_steps) {
    String[] arr = new String[path_steps.length + 1];
    arr[0] = system_name;
    for (int i = 0; i < path_steps.length; i++) {
      arr[i + 1] = path_steps[i];
    }
    return LOADER.getStringValue(arr);
  }

  protected final boolean getBoolConstant(String... path_steps) {
    String[] arr = new String[path_steps.length + 1];
    arr[0] = system_name;
    for (int i = 0; i < path_steps.length; i++) {
      arr[i + 1] = path_steps[i];
    }
    return LOADER.getBoolValue(arr);
  }
}
