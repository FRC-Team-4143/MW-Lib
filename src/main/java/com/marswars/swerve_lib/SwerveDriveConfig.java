package com.marswars.swerve_lib;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.marswars.mechanisms.MotorConfig;
import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.swerve_lib.module.ModuleType;
import com.marswars.swerve_lib.module.SwerveModuleConfig;
import com.marswars.util.PhoenixUtil;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Immutable configuration container for a full swerve drivetrain.
 */
public class SwerveDriveConfig {
    public final SwerveModuleConfig FL_MODULE_CONSTANTS;
    public final SwerveModuleConfig FR_MODULE_CONSTANTS;
    public final SwerveModuleConfig BL_MODULE_CONSTANTS;
    public final SwerveModuleConfig BR_MODULE_CONSTANTS;

    public final int PIGEON2_ID;
    public final String PIGEON2_CANBUS_NAME;

    /**
     * Default skid-detection threshold (max allowed spread of per-module velocities) used
     * when a configuration does not specify one.
     */
    public static final double DEFAULT_SKID_DETECTION_RANGE = 0.3;

    /** Skid-detection threshold (max allowed spread of per-module velocities). */
    public final double SKID_DETECTION_RANGE;

    /**
     * Creates a new swerve drive configuration with the default skid-detection threshold.
     *
     * @param fl_module_constants Front-left module configuration
     * @param fr_module_constants Front-right module configuration
     * @param bl_module_constants Back-left module configuration
     * @param br_module_constants Back-right module configuration
     * @param pigeon2_id Pigeon2 CAN device ID
     * @param pigeon2_canbus_name Pigeon2 CAN bus name
     */
    public SwerveDriveConfig(
            SwerveModuleConfig fl_module_constants,
            SwerveModuleConfig fr_module_constants,
            SwerveModuleConfig bl_module_constants,
            SwerveModuleConfig br_module_constants,
            int pigeon2_id,
            String pigeon2_canbus_name) {
        this(
                fl_module_constants,
                fr_module_constants,
                bl_module_constants,
                br_module_constants,
                pigeon2_id,
                pigeon2_canbus_name,
                DEFAULT_SKID_DETECTION_RANGE);
    }

    /**
     * Creates a new swerve drive configuration.
     *
     * @param fl_module_constants Front-left module configuration
     * @param fr_module_constants Front-right module configuration
     * @param bl_module_constants Back-left module configuration
     * @param br_module_constants Back-right module configuration
     * @param pigeon2_id Pigeon2 CAN device ID
     * @param pigeon2_canbus_name Pigeon2 CAN bus name
     * @param skid_detection_range Skid-detection threshold (max allowed spread of
     *     per-module velocities)
     */
    public SwerveDriveConfig(
            SwerveModuleConfig fl_module_constants,
            SwerveModuleConfig fr_module_constants,
            SwerveModuleConfig bl_module_constants,
            SwerveModuleConfig br_module_constants,
            int pigeon2_id,
            String pigeon2_canbus_name,
            double skid_detection_range) {
        FL_MODULE_CONSTANTS = fl_module_constants;
        FR_MODULE_CONSTANTS = fr_module_constants;
        BL_MODULE_CONSTANTS = bl_module_constants;
        BR_MODULE_CONSTANTS = br_module_constants;

        PIGEON2_ID = pigeon2_id;
        PIGEON2_CANBUS_NAME = pigeon2_canbus_name;
        SKID_DETECTION_RANGE = skid_detection_range;
    }

    /** Starts a {@link Builder} for a four-module TalonFX swerve drivetrain. */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Builds a {@link SwerveDriveConfig} from the values that actually vary between robots. Team
     * conventions are filled in: brake mode, motor inversion, steer inversion and gear ratios from
     * the {@link ModuleType}, and the Pigeon2 on the module CAN bus.
     *
     * <pre>{@code
     * SwerveDriveConfig.builder()
     *         .canbus("CANivore")
     *         .moduleType(ModuleType.getModuleType("TSN-P13-S18"))
     *         .wheelRadius(Units.inchesToMeters(1.8))
     *         .speedAt12V(5.0)
     *         .driveGains(drive_slot0, drive_slot1)
     *         .steerMotor(TalonMotorType.X44)
     *         .steerGains(steer_slot0, steer_slot1)
     *         .frontLeft(1, 2, 0, new Translation2d(0.19, 0.19))
     *         // frontRight, backLeft, backRight ...
     *         .build();
     * }</pre>
     */
    public static class Builder {
        /** Supply current is lowered to the lower limit after exceeding the limit this long. */
        public static final double DEFAULT_SUPPLY_CURRENT_LOWER_TIME = 0.1;

        private String canbus_name = "rio";
        private ModuleType module_type;
        private double wheel_radius_m = Double.NaN;
        private double speed_at_12_volts = Double.NaN;
        private SwerveModuleConfig.EncoderType encoder_type =
                SwerveModuleConfig.EncoderType.ANALOG_ENCODER;
        private boolean enable_foc = false;

        private TalonMotorType drive_motor_type = TalonMotorType.X60;
        private Slot0Configs drive_slot0 = new Slot0Configs();
        private Slot1Configs drive_slot1 = new Slot1Configs();
        private double drive_supply_limit = 40.0;
        private double drive_stator_limit = 40.0;

        private TalonMotorType steer_motor_type = TalonMotorType.X60;
        private Slot0Configs steer_slot0 = new Slot0Configs();
        private Slot1Configs steer_slot1 = new Slot1Configs();
        private double steer_supply_limit = 40.0;
        private double steer_stator_limit = 40.0;

        private int pigeon2_id = 0;
        private double skid_detection_range = DEFAULT_SKID_DETECTION_RANGE;

        private final int[][] module_ids = new int[4][];
        private final Translation2d[] module_locations = new Translation2d[4];
        private final boolean[] drive_inverted = new boolean[4];

        private Builder() {}

        /** CAN bus for the modules and the Pigeon2 (default "rio"). */
        public Builder canbus(String canbus_name) {
            this.canbus_name = canbus_name;
            return this;
        }

        /** Module gearing preset; also supplies the gear ratios and steer inversion. Required. */
        public Builder moduleType(ModuleType module_type) {
            this.module_type = module_type;
            return this;
        }

        /** Drive wheel radius in meters. Required. */
        public Builder wheelRadius(double wheel_radius_m) {
            this.wheel_radius_m = wheel_radius_m;
            return this;
        }

        /** Free speed at 12 V in meters per second (drive feedforward). Required. */
        public Builder speedAt12V(double speed_at_12_volts) {
            this.speed_at_12_volts = speed_at_12_volts;
            return this;
        }

        /** Steer absolute encoder hardware (default analog encoder). */
        public Builder encoderType(SwerveModuleConfig.EncoderType encoder_type) {
            this.encoder_type = encoder_type;
            return this;
        }

        /** Use FOC (torque-current) control requests (default false). */
        public Builder enableFoc(boolean enable_foc) {
            this.enable_foc = enable_foc;
            return this;
        }

        /** Drive motor type (default X60). */
        public Builder driveMotor(TalonMotorType motor_type) {
            this.drive_motor_type = motor_type;
            return this;
        }

        /** Drive gains: slot 0 is position, slot 1 is velocity. Shared by all modules. */
        public Builder driveGains(Slot0Configs slot0, Slot1Configs slot1) {
            this.drive_slot0 = slot0;
            this.drive_slot1 = slot1;
            return this;
        }

        /** Drive supply and stator current limits in amps (default 40 / 40). */
        public Builder driveCurrentLimits(double supply_amps, double stator_amps) {
            this.drive_supply_limit = supply_amps;
            this.drive_stator_limit = stator_amps;
            return this;
        }

        /** Steer motor type (default X60). */
        public Builder steerMotor(TalonMotorType motor_type) {
            this.steer_motor_type = motor_type;
            return this;
        }

        /** Steer gains (slot 0 is used for position control). Shared by all modules. */
        public Builder steerGains(Slot0Configs slot0, Slot1Configs slot1) {
            this.steer_slot0 = slot0;
            this.steer_slot1 = slot1;
            return this;
        }

        /** Steer supply and stator current limits in amps (default 40 / 40). */
        public Builder steerCurrentLimits(double supply_amps, double stator_amps) {
            this.steer_supply_limit = supply_amps;
            this.steer_stator_limit = stator_amps;
            return this;
        }

        /** Pigeon2 CAN ID (default 0); it rides the module CAN bus. */
        public Builder pigeon2Id(int pigeon2_id) {
            this.pigeon2_id = pigeon2_id;
            return this;
        }

        /** Max allowed spread of per-module velocities before skidding is flagged. */
        public Builder skidDetectionRange(double skid_detection_range) {
            this.skid_detection_range = skid_detection_range;
            return this;
        }

        /** Front-left module CAN IDs and location (meters, +x forward, +y left). */
        public Builder frontLeft(int drive_id, int steer_id, int encoder_id, Translation2d location) {
            return module(0, drive_id, steer_id, encoder_id, location, false);
        }

        /** Front-right module CAN IDs and location (meters, +x forward, +y left). */
        public Builder frontRight(int drive_id, int steer_id, int encoder_id, Translation2d location) {
            return module(1, drive_id, steer_id, encoder_id, location, false);
        }

        /** Back-left module CAN IDs and location (meters, +x forward, +y left). */
        public Builder backLeft(int drive_id, int steer_id, int encoder_id, Translation2d location) {
            return module(2, drive_id, steer_id, encoder_id, location, false);
        }

        /** Back-right module CAN IDs and location (meters, +x forward, +y left). */
        public Builder backRight(int drive_id, int steer_id, int encoder_id, Translation2d location) {
            return module(3, drive_id, steer_id, encoder_id, location, false);
        }

        /**
         * Sets a module by index (0 FL, 1 FR, 2 BL, 3 BR), with an explicit drive inversion for
         * modules mounted mirrored.
         */
        public Builder module(
                int index,
                int drive_id,
                int steer_id,
                int encoder_id,
                Translation2d location,
                boolean invert_drive) {
            module_ids[index] = new int[] {drive_id, steer_id, encoder_id};
            module_locations[index] = location;
            drive_inverted[index] = invert_drive;
            return this;
        }

        /** Builds the drivetrain config, creating fresh motor configs for every module. */
        public SwerveDriveConfig build() {
            if (module_type == null) {
                throw new IllegalStateException("SwerveDriveConfig.Builder: moduleType is required");
            }
            if (Double.isNaN(wheel_radius_m) || Double.isNaN(speed_at_12_volts)) {
                throw new IllegalStateException(
                        "SwerveDriveConfig.Builder: wheelRadius and speedAt12V are required");
            }
            SwerveModuleConfig[] modules = new SwerveModuleConfig[4];
            for (int i = 0; i < 4; i++) {
                if (module_ids[i] == null) {
                    throw new IllegalStateException(
                            "SwerveDriveConfig.Builder: module " + i + " (FL, FR, BL, BR) not set");
                }
                modules[i] = buildModule(i);
            }
            return new SwerveDriveConfig(
                    modules[0],
                    modules[1],
                    modules[2],
                    modules[3],
                    pigeon2_id,
                    canbus_name,
                    skid_detection_range);
        }

        private SwerveModuleConfig buildModule(int i) {
            SwerveModuleConfig module = new SwerveModuleConfig();
            module.module_type = module_type;
            module.encoder_type = encoder_type;
            module.encoder_id = module_ids[i][2];
            module.wheel_radius_m = wheel_radius_m;
            module.speed_at_12_volts = speed_at_12_volts;
            module.location_x = module_locations[i].getX();
            module.location_y = module_locations[i].getY();
            module.enable_foc = enable_foc;
            module.drive_motor_config =
                    motor(
                            module_ids[i][0],
                            drive_motor_type,
                            drive_inverted[i],
                            drive_slot0,
                            drive_slot1,
                            drive_supply_limit,
                            drive_stator_limit);
            // Steer inversion is overwritten from the ModuleType when the module is constructed
            module.steer_motor_config =
                    motor(
                            module_ids[i][1],
                            steer_motor_type,
                            module_type.steerInverted,
                            steer_slot0,
                            steer_slot1,
                            steer_supply_limit,
                            steer_stator_limit);
            return module;
        }

        private MotorConfig motor(
                int can_id,
                TalonMotorType motor_type,
                boolean inverted,
                Slot0Configs slot0,
                Slot1Configs slot1,
                double supply_limit,
                double stator_limit) {
            MotorConfig motor = new MotorConfig();
            motor.canbus_name = canbus_name;
            motor.can_id = can_id;
            motor.motor_type = motor_type;
            // A fresh configuration per motor: ModuleTalonFX mutates it (gear ratios). The gain
            // objects are shared and only read downstream.
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.Inverted = PhoenixUtil.toInvertedValue(inverted);
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.Slot0 = slot0;
            config.Slot1 = slot1;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = supply_limit;
            config.CurrentLimits.SupplyCurrentLowerTime = DEFAULT_SUPPLY_CURRENT_LOWER_TIME;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = stator_limit;
            motor.apply(config);
            return motor;
        }
    }
}
