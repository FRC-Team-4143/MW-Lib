package com.marswars.subsystem;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;

/**
 * Serves the subsystem console, a single web page bundled in MW-Lib, from the robot. The page
 * talks NT4 to the robot from the browser and shows every subsystem's control, each IO's on/off
 * switch, sensor overrides and manual output. It does not edit gains; use your usual dashboard for
 * {@code /Tuning}.
 *
 * <pre>{@code
 * // RobotContainer constructor
 * startSubsystemConsole(SubsystemConsole.DEFAULT_PORT);
 * // then browse to http://10.TE.AM.2:5807/ (or http://localhost:5807/ in simulation)
 * }</pre>
 */
public final class SubsystemConsole {
    /** The console's default HTTP port. */
    public static final int DEFAULT_PORT = 5807;

    private static final String RESOURCE = "/com/marswars/subsystem/console/index.html";
    private static boolean started_ = false;

    private SubsystemConsole() {}

    /**
     * Copies the page out of the jar into a temporary folder and serves it. Safe to call more than
     * once; only the first call starts a server.
     *
     * @param port the HTTP port
     */
    public static synchronized void start(int port) {
        if (started_) {
            return;
        }
        try (InputStream page = SubsystemConsole.class.getResourceAsStream(RESOURCE)) {
            if (page == null) {
                DataLogManager.log("Subsystem console page missing from the MW-Lib jar");
                return;
            }
            Path dir = Files.createTempDirectory("mwlib-console");
            Files.copy(page, dir.resolve("index.html"), StandardCopyOption.REPLACE_EXISTING);
            WebServer.start(port, dir.toString());
            started_ = true;
            DataLogManager.log("Subsystem console at http://<robot>:" + port + "/");
        } catch (IOException e) {
            DataLogManager.log("Subsystem console failed to start: " + e.getMessage());
        }
    }
}
