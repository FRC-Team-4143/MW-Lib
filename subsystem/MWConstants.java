package frc.mw_lib.subsystem;

import frc.mw_lib.util.ConstantsLoader;
import frc.mw_lib.util.MWPreferences;

public class MWConstants {
  private final String system_name;

  protected MWConstants() {
    // Use some Java magic to pull the class name
    String name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);
    if (name.endsWith("Constants")) {
      name = name.substring(0, name.length() - "Constants".length());
    }

    system_name = name.toLowerCase();
  }

  // ConstantsLoader instance for loading configuration values
  protected final ConstantsLoader LOADER = ConstantsLoader.getInstance();

  // MW Preferences loader for loading from and to preferences
  protected final MWPreferences PREFERENCES = MWPreferences.getInstance();

  protected final double getDoubleConstant(String... path_steps) {
    String[] arr = new String[path_steps.length + 1];
    arr[0] = system_name;
    for (int i = 0; i < path_steps.length; i++) {
      arr[i + 1] = path_steps[i];
    }
    return LOADER.getDoubleValue(path_steps);
  }

  protected final int getIntConstant(String... path_steps) {
    String[] arr = new String[path_steps.length + 1];
    arr[0] = system_name;
    for (int i = 0; i < path_steps.length; i++) {
      arr[i + 1] = path_steps[i];
    }
    return LOADER.getIntValue(path_steps);
  }

  protected final String getStringConstant(String... path_steps) {
    String[] arr = new String[path_steps.length + 1];
    arr[0] = system_name;
    for (int i = 0; i < path_steps.length; i++) {
      arr[i + 1] = path_steps[i];
    }
    return LOADER.getStringValue(path_steps);
  }
}
