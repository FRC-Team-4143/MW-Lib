package com.marswars.swerve_lib.module;

import edu.wpi.first.wpilibj.DataLogManager;
import com.marswars.util.ConstantsLoader;
import java.util.Hashtable;

public class ModuleType {
    public final String name;
    public final double steerRatio;
    public final double driveRatio;
    public final boolean steerInverted;

    private static final ConstantsLoader LOADER = ConstantsLoader.getInstance();

    ModuleType(String name, double steerRatio, double driveRatio, boolean steerInverted) {
        this.name = name;
        this.steerRatio = steerRatio;
        this.driveRatio = driveRatio;
        this.steerInverted = steerInverted;
    }

    private static final ModuleType[] ALL_TYPES = {
        // MK4I
        // https://www.swervedrivespecialties.com/collections/kits/products/mk4i-swerve-module
        new ModuleType("MK4I-L1", 150.0 / 7.0, 8.14, true),
        new ModuleType("MK4I-L2", 150.0 / 7.0, 6.75, true),
        new ModuleType("MK4I-L3", 150.0 / 7.0, 6.12, true),

        // MK4I Adapter 16T Pinion Kit
        // https://www.swervedrivespecialties.com/products/kit-adapter-16t-drive-pinion-gear-mk4i
        new ModuleType("MK4I-L1+", 150.0 / 7.0, 7.13, true),
        new ModuleType("MK4I-L2+", 150.0 / 7.0, 5.9, true),
        new ModuleType("MK4I-L3+", 150.0 / 7.0, 5.36, true),

        // MK4N
        // https://www.swervedrivespecialties.com/collections/kits/products/mk4n-swerve-module
        new ModuleType("MK4N-L1+", 18.75, 7.13, true),
        new ModuleType("MK4N-L2+", 18.75, 5.9, true),
        new ModuleType("MK4N-L3+", 18.75, 5.36, true),

        // MKC
        // https://www.swervedrivespecialties.com/collections/kits/products/mk4c-swerve-module
        new ModuleType("MK4C-L1+", 12.8, 7.13, false),
        new ModuleType("MK4C-L2+", 12.8, 5.9, false),
        new ModuleType("MK4C-L3+", 12.8, 5.36, false),

        // MK4
        // https://www.swervedrivespecialties.com/products/mk4-swerve-module
        new ModuleType("MK4-L1", 12.8, 8.14, false),
        new ModuleType("MK4-L2", 12.8, 6.75, false),
        new ModuleType("MK4-L3", 12.8, 6.12, false),
        new ModuleType("MK4-L4", 12.8, 5.14, false),

        // TSN
        new ModuleType("TSN-P12-S18", 24, 6.74, true),
        new ModuleType("TSN-P13-S18", 24, 6.23, true),
        new ModuleType("TSN-P14-S18", 24, 5.79, true),
        new ModuleType("TSN-P12-S16", 24, 6.00, true),
        new ModuleType("TSN-P13-S16", 24, 5.54, true),
        new ModuleType("TSN-P14-S16", 24, 5.14, true),
    };

    public static Hashtable<String, ModuleType> ALL_MODULE_TYPES = new Hashtable<>();

    static {
        for (ModuleType type : ALL_TYPES) {
            ALL_MODULE_TYPES.put(type.name, type);
        }
    }

    /**
     * @param position String representing the module location [fl, fr, bl, br, etc..]
     * @return ModuleType to be load the gear ratio constants from
     */
    public static ModuleType getModuleTypeFromJSON(String position) {
        String type = LOADER.getStringValue("swerve", position, "module_type");
        String gearing = LOADER.getStringValue("swerve", position, "module_gearing");
        return getModuleType(type + "-" + gearing);
    }

    public static ModuleType getModuleType(String type) {
        ModuleType module = ALL_MODULE_TYPES.get(type);
        if (module == null) {
            DataLogManager.log("ERROR: Invalid Module Type: " + type + "\nDefaulting to MK4I-L3");
            module = ALL_MODULE_TYPES.get("MK4I-L3");
        }
        return module;
    }
}
