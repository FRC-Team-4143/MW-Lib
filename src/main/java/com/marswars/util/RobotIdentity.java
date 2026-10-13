package com.marswars.util;

import com.marswars.logging.MwLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Resolves which physical robot the code is running on.
 *
 * <p>On a real robot the name comes from the persistent "RobotName" preference burned onto the
 * RoboRIO via the Test-mode "Config/Burn RobotName" dashboard button. In simulation the name
 * defaults to "SimBot" and can be overridden with the ROBOT_NAME environment variable.
 */
public class RobotIdentity {
    private static RobotIdentity instance_;

    public static RobotIdentity getInstance() {
        if (instance_ == null) {
            instance_ = new RobotIdentity();
        }

        return instance_;
    }

    private static final String robot_name_pref_name = "RobotName";
    private final String robot_name_;

    private RobotIdentity() {
        String robot_name = "AlphaBot";
        if (RobotBase.isSimulation()) {
            // Default to SimBot; during replay the name recorded in the log wins, and the
            // ROBOT_NAME environment variable explicitly overrides both
            robot_name = "SimBot";
            String log_name = MwLog.readReplayMetadata("RobotName");
            String env_name = System.getenv("ROBOT_NAME");
            if (env_name != null) {
                robot_name = env_name;
                if (log_name != null && !log_name.equals(env_name)) {
                    DriverStation.reportWarning(
                            "ROBOT_NAME=" + env_name + " overrides the replay log's robot: "
                                    + log_name,
                            false);
                }
                DriverStation.reportWarning(
                        "Simulation Environment Detected, Using Robot Name: " + robot_name, false);
            } else if (log_name != null) {
                robot_name = log_name;
                DriverStation.reportWarning(
                        "Replay Detected, Using Robot Name from log: " + robot_name, false);
            } else {
                DriverStation.reportWarning(
                        "Simulation Environment Detected, Using Robot Name: " + robot_name, false);
            }
        } else if (MWPreferences.getInstance().hasPreference(robot_name_pref_name)) {
            robot_name =
                    MWPreferences.getInstance()
                            .getPreferenceString(robot_name_pref_name, robot_name);
        } else {
            DriverStation.reportError(
                    "Failed to retrieve robot name on startup, using default: " + robot_name,
                    false);
        }

        // record the final robot name
        robot_name_ = robot_name;

        SmartDashboard.putString("Config/RobotName", robot_name);
        // Burn Robot Name Command
        SmartDashboard.putData(
                "Config/Burn RobotName",
                Commands.runOnce(() -> burnRobotName())
                        .onlyIf(RobotState::isTest)
                        .ignoringDisable(true));
    }

    public void burnRobotName() {
        String robot_name = SmartDashboard.getString("Config/RobotName", "");
        if (!robot_name.isBlank()) {
            MWPreferences.getInstance().setPreference(robot_name_pref_name, robot_name);
            DataLogManager.log("Updated RobotName to " + robot_name + " - Restart Robot Code!!!!");
        } else {
            DataLogManager.log("Cannot Configure Robot with Blank Name");
        }
    }

    public String getRobotName() {
        return robot_name_;
    }

    /**
     * Maps the robot name onto one of the robot project's IDs, e.g.
     *
     * <pre>{@code
     * public enum Robot { ALPHA_BOT, BETA_BOT, SIM_BOT }
     * public static final Robot ROBOT = RobotIdentity.getInstance().getRobot(Robot.class);
     * }</pre>
     *
     * Constant names match case-insensitively and ignoring underscores ("BetaBot" matches {@code
     * BETA_BOT}). An unknown name reports an error and falls back to the first constant.
     *
     * @param robots the robot project's enum of robots
     * @return the constant for the robot the code is running on
     */
    public <E extends Enum<E>> E getRobot(Class<E> robots) {
        E robot = match(robot_name_, robots);
        if (robot == null) {
            robot = robots.getEnumConstants()[0];
            DriverStation.reportError(
                    "Unknown robot name '" + robot_name_ + "', defaulting to " + robot.name(),
                    false);
        }
        DataLogManager.log("Robot: " + robot.name());
        return robot;
    }

    /** The constant whose name matches {@code name} ignoring case and underscores, or null. */
    static <E extends Enum<E>> E match(String name, Class<E> robots) {
        String wanted = normalize(name);
        for (E robot : robots.getEnumConstants()) {
            if (normalize(robot.name()).equals(wanted)) {
                return robot;
            }
        }
        return null;
    }

    private static String normalize(String name) {
        return name.replace("_", "").toLowerCase();
    }
}
