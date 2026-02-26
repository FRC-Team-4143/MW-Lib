package com.marswars.proxy_server;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

import com.marswars.data_structures.ConcurrentFifoQueue;
import com.marswars.vision.MwVisionSim;
import com.marswars.proxy_server.OdomPacket.OdometryData;
import com.marswars.proxy_server.PieceDetectionPacket.PieceDetectionData;
import com.marswars.proxy_server.StatesPacket.ModuleStatesData;
import com.marswars.proxy_server.TagSolutionPacket.TagSolutionData;
import com.marswars.proxy_server.TimesyncRequest.TimesyncRequestData;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.OptionalInt;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Proxy server thread for handling UDP communication with external systems.
 * Receives and processes various packet types including odometry, module states,
 * AprilTag solutions, piece detections, and timesync requests.
 * Provides bidirectional communication capabilities on a single UDP port.
 * 
 * <h2>Multi-Client Support</h2>
 * The proxy server can handle multiple simultaneous clients (e.g., multiple vision coprocessors).
 * Each client is tracked independently with its own connection monitoring and alerts.
 * Data from all clients is merged into shared queues, accessible via the getter methods.
 * <p>
 * Use {@link #broadcastData(byte[])} to send data to all connected clients, or 
 * {@link #sendDataToClient(byte[], SocketAddress)} to send to a specific client.
 * The legacy {@link #sendData(byte[])} method sends to the most recent client for 
 * backward compatibility.
 * 
 * <h2>Connection Monitoring</h2>
 * The proxy server automatically monitors the connection to each client and publishes
 * individual WPILib Alerts when connections are lost. Connection is considered lost if no
 * packets are received within the timeout period (1 second by default). A debouncer
 * is used to prevent alert flapping during intermittent connectivity issues.
 * <p>
 * Use {@link #isConnected()} to check if any client is connected,
 * {@link #getConnectedClientCount()} to get the number of active clients, and
 * {@link #getTimeSinceLastPacket()} to get time since last received packet from any client.
 * 
 * <h2>PhotonVision Simulation Support</h2>
 * In simulation mode, this class can use PhotonVision simulation to generate 
 * simulated vision data that populates the tagSolutions queue, allowing you to 
 * test vision-based robot code without a physical robot or vision coprocessor.
 * 
 * <h3>Usage Example:</h3>
 * <pre>{@code
 * // In your robot initialization (Robot.java or subsystem)
 * ProxyServerThread proxyServer = ProxyServerThread.getInstance();
 * 
 * // Check connection status for multiple clients
 * int connectedClients = proxyServer.getConnectedClientCount();
 * System.out.println("Connected clients: " + connectedClients);
 * 
 * // Get all client addresses
 * List<SocketAddress> clients = proxyServer.getClientAddresses();
 * 
 * // Broadcast data to all connected clients
 * byte[] customData = new byte[]{1, 2, 3};
 * int sentCount = proxyServer.broadcastData(customData);
 * System.out.println("Sent to " + sentCount + " clients");
 * 
 * // Initialize vision simulation (only in simulation mode)
 * if (RobotBase.isSimulation()) {
 *     // Load the field layout
 *     AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
 *         AprilTagFields.k2025Reefscape
 *     );
 *     MwVisionSimulation visionSim = proxyServer.initializeVisionSimulation(fieldLayout);
 *     
 *     // Add default cameras (front, left, right, back)
 *     visionSim.addDefaultCameras();
 *     
 *     // Or add a custom camera
 *     // Camera mounted 0.5m forward, 0.3m up, angled 15 degrees down
 *     Transform3d robotToCamera = new Transform3d(
 *         new Translation3d(0.5, 0, 0.3),
 *         new Rotation3d(0, Math.toRadians(-15), 0)
 *     );
 *     visionSim.addCamera("simCamera", robotToCamera);
 * }
 * 
 * // In your periodic method, update with current robot pose
 * if (RobotBase.isSimulation()) {
 *     Pose2d currentPose = drivetrain.getPose(); // Get from your drivetrain
 *     proxyServer.updateVisionSimulation(currentPose);
 * }
 * 
 * // Vision data will now be available via getLatestTagSolutions()
 * List<TagSolutionData> solutions = proxyServer.getLatestTagSolutions();
 * }</pre>
 * 
 * @see com.marswars.vision.MwVisionSim
 * @see TagSolutionPacket.TagSolutionData
 */
public class ProxyServerThread extends Thread {

    // Data packet queues for storing received information
    private final ConcurrentFifoQueue<OdometryData> odometry_readings_ = new ConcurrentFifoQueue<>(20);
    private final ConcurrentFifoQueue<ModuleStatesData> module_states_readings_ = new ConcurrentFifoQueue<>(20);
    private final ConcurrentFifoQueue<TagSolutionData> tag_solutions_ = new ConcurrentFifoQueue<>(20);
    private final ConcurrentFifoQueue<PieceDetectionData> piece_detections_ = new ConcurrentFifoQueue<>(20);

    // Socket Config
    private final int PORT = 5809; // local port to bind server
    private DatagramSocket socket_ = null;
    private final int TIMEOUT = 1; // Server receive blocking timeout
    
    // Multi-client connection tracking
    private static final double CONNECTION_TIMEOUT_SECONDS = 1.0; // Consider disconnected after 1 second
    private final Map<String, ClientConnection> clients_ = new ConcurrentHashMap<>();
    private SocketAddress last_client_address_ = null; // For backward compatibility with sendData()
    
    // Initial connection tracking
    private boolean has_ever_connected_ = false; // Track if any client has ever connected
    private final Alert no_clients_alert_ = new Alert("Proxy Server: No clients connected", AlertType.kError);
    
    /**
     * Tracks connection state for an individual client
     */
    private static class ClientConnection {
        final SocketAddress address;
        double last_packet_time;
        final Debouncer debouncer;
        boolean connected;
        final Alert alert;
        
        ClientConnection(SocketAddress address) {
            this.address = address;
            this.last_packet_time = Timer.getFPGATimestamp();
            this.debouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);
            this.connected = true; // Start as connected when first packet received
            // Create alert with client-specific name
            String client_name = address.toString();
            this.alert = new Alert("Proxy Server: Lost connection to " + client_name, AlertType.kError);
        }
        
        void updatePacketReceived() {
            this.last_packet_time = Timer.getFPGATimestamp();
        }
        
        void updateConnectionStatus() {
            double current_time = Timer.getFPGATimestamp();
            double time_since_last_packet = current_time - last_packet_time;
            boolean is_receiving = time_since_last_packet < CONNECTION_TIMEOUT_SECONDS;
            connected = debouncer.calculate(is_receiving);
            alert.set(!connected);
        }
    }
    
    // Vision simulation (only used in simulation mode)
    private MwVisionSim visionSim = null;

    // Singleton instance
    private static ProxyServerThread instance_ = null;

    /**
     * Gets the singleton instance of the ProxyServerThread.
     *
     * @return the ProxyServerThread instance
     */
    public static ProxyServerThread getInstance() {
        if (instance_ == null) {
            instance_ = new ProxyServerThread();
            instance_.start();
        }
        return instance_;
    }

    @Override
    public void start(){
        instance_.configureServer();
        super.start();
    }

    @Override
    public void run(){
        while (true) {
            updateData();
            updateConnectionStatus();
        }
    }

    /**
     * Binds the server socket to the set port to begin communication. This must be called once
     * before you can attempt to {@link #updateData()}.
     *
     * @return true: server configured successfully | false: configuration error occurred.
     * @throws SocketException if the socket could not be opened, or the socket could not bind to
     *     the specified local port.
     */
    public boolean configureServer() {
        // check if socket is already bound
        if (socket_ == null || !socket_.isBound()) {
            try {
                socket_ = new DatagramSocket(PORT);
                // set receive blocking timeout (ms)
                socket_.setSoTimeout(TIMEOUT);
            } catch (SocketException e) {
                e.printStackTrace();
                return false;
            }
            // socket configured successfully
            return true;
        }
        // socket already configured
        return true;
    }

    /**
     * Server attempts to receive data from bound socket and parse the incoming packet. This method
     * is called periodically to continuously update internal members. Ensure {@link
     * #configureServer()}
     *
     * @return true: packet successfully received | false: receive error occurred.
     * @throws SocketTimeoutException if socket receive timed out to avoid blocking.
     * @throws IOException if I/O error occurs.
     */
    private boolean updateData() {

        try {
            // clear the buffer after every message
            byte[] buffer = new byte[45];

            // create a packet to receive the data
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

            // receive the data in byte buffer
            socket_.receive(packet);
            
            // Track client connection
            SocketAddress client_address = packet.getSocketAddress();
            last_client_address_ = client_address; // For backward compatibility with sendData()
            String client_key = client_address.toString();
            
            // Mark that we've had at least one client connection
            if (!has_ever_connected_) {
                has_ever_connected_ = true;
            }
            
            // Get or create client connection tracker
            ClientConnection client = clients_.computeIfAbsent(client_key, 
                k -> new ClientConnection(client_address));
            client.updatePacketReceived();

            // Determine packet type from first byte of buffer
            switch ((int) buffer[Packet.PACKET_ID_IDX]) {
                // Odometry Packet Type
                case OdomPacket.TYPE_ID: // const uint8_t msg_id{ 30u };
                    odometry_readings_.add(OdomPacket.updateData(buffer));
                    break;
                // Module States Packet Type
                case StatesPacket.TYPE_ID: // const uint8_t msg_id{ 2u };
                    module_states_readings_.add(StatesPacket.updateData(buffer));
                    break;
                // AprilTag Solution Packet Type
                case TagSolutionPacket.TYPE_ID: // const uint8_t msg_id{ 15u };
                    tag_solutions_.add(TagSolutionPacket.updateData(buffer));
                    break;
                // Piece Detection Packet Type
                case PieceDetectionPacket.TYPE_ID: // const uint8_t msg_id{ 10u };
                    piece_detections_.add(PieceDetectionPacket.updateData(buffer));
                    break;
                // Timesync Request Packet Type
                case TimesyncRequest.TYPE_ID: // const uint8_t msg_id{ 60u };
                    // Parse request and send response
                    TimesyncRequestData timesync_request = TimesyncRequest.updateData(buffer);
                    sendTimesyncResponse(timesync_request);
                    break;
                // Unknown Packet Type
                default:
                    return false;
            }

        } catch (SocketTimeoutException e) {
            // Timeout occurred
            return false;
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }

        // packet was processed correctly
        return true;
    }

    /**
     * Updates the connection status for all tracked clients.
     * Uses a debouncer to prevent alert flapping and sets a warning alert when
     * connection is lost. Should be called periodically from the run loop.
     */
    private void updateConnectionStatus() {
        // Update connection status for each client
        for (ClientConnection client : clients_.values()) {
            client.updateConnectionStatus();
        }
        
        // Show "no clients" alert only if no client has ever connected
        // Once a client connects, switch to per-client disconnection alerts
        no_clients_alert_.set(!has_ever_connected_ && clients_.isEmpty());
    }

    /**
     * Checks if the proxy server currently has an active connection to any client.
     * 
     * @return true if at least one client is connected, false if all connections lost
     */
    public boolean isConnected() {
        return clients_.values().stream().anyMatch(c -> c.connected);
    }

    /**
     * Gets the number of currently connected clients.
     * 
     * @return number of clients with active connections
     */
    public int getConnectedClientCount() {
        return (int) clients_.values().stream().filter(c -> c.connected).count();
    }

    /**
     * Gets a list of all tracked client addresses (both connected and disconnected).
     * 
     * @return list of client socket addresses
     */
    public List<SocketAddress> getClientAddresses() {
        return new ArrayList<>(clients_.values().stream()
            .map(c -> c.address)
            .toList());
    }

    /**
     * Gets the time elapsed since the last packet was received from any client.
     * 
     * @return seconds since last packet from any client, or Double.MAX_VALUE if no clients
     */
    public double getTimeSinceLastPacket() {
        return clients_.values().stream()
            .mapToDouble(c -> Timer.getFPGATimestamp() - c.last_packet_time)
            .min()
            .orElse(Double.MAX_VALUE);
    }

    /**
     * Sends data back to the last connected client on the same port.
     * For multi-client support, use {@link #sendDataToClient(byte[], SocketAddress)} instead.
     * 
     * @param buffer the byte array to send
     * @return true if data was sent successfully, false if there was an error
     */
    public boolean sendData(byte[] buffer) {
        if (socket_ == null || !socket_.isBound()) {
            System.err.println("Socket not configured. Call configureServer() first.");
            return false;
        }
        
        if (last_client_address_ == null) {
            System.err.println("No client address available. Must receive data first.");
            return false;
        }
        
        return sendDataToClient(buffer, last_client_address_);
    }

    /**
     * Sends data to a specific client address.
     * 
     * @param buffer the byte array to send
     * @param clientAddress the destination client address
     * @return true if data was sent successfully, false if there was an error
     */
    public boolean sendDataToClient(byte[] buffer, SocketAddress client_address) {
        if (socket_ == null || !socket_.isBound()) {
            System.err.println("Socket not configured. Call configureServer() first.");
            return false;
        }
        
        if (client_address == null) {
            System.err.println("Client address cannot be null.");
            return false;
        }
        
        try {
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length, client_address);
            socket_.send(packet);
            return true;
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
    }

    /**
     * Broadcasts data to all currently connected clients.
     * 
     * @param buffer the byte array to send
     * @return number of clients successfully sent to
     */
    public int broadcastData(byte[] buffer) {
        int success_count = 0;
        for (ClientConnection client : clients_.values()) {
            if (client.connected && sendDataToClient(buffer, client.address)) {
                success_count++;
            }
        }
        return success_count;
    }

    /**
     * Gets the current robot pose, velocity, and variance data from odometry readings.
     * Data is updated by calling {@link #updateData()} periodically.
     *
     * @return list of recent {@link OdometryData} from chassis proxy
     */
    public List<OdometryData> getLatestOdometryReadings() {
        return odometry_readings_.toList();
    }

    /**
     * Gets the current robot module states information.
     * States are updated by calling {@link #updateData()} periodically.
     *
     * @return list of recent {@link ModuleStatesData} containing swerve module information
     */
    public List<ModuleStatesData> getLatestModuleStatesReadings() {
        return module_states_readings_.toList();
    }

    /**
     * Gets the current robot module states as individual SwerveModuleState arrays.
     * This is a convenience method that extracts the module states arrays from the data objects.
     * 
     * @return list of SwerveModuleState arrays (for backward compatibility)
     */
    public List<SwerveModuleState[]> getLatestModuleStatesArrays() {
        List<SwerveModuleState[]> module_states_arrays = new ArrayList<>();
        for (ModuleStatesData states_data : module_states_readings_.toList()) {
            module_states_arrays.add(states_data.moduleStates);
        }
        return module_states_arrays;
    }

    /**
     * Gets the current AprilTag solution information.
     * Solutions are updated by calling {@link #updateData()} periodically.
     *
     * @return list of recent {@link TagSolutionData} with pose estimates and detected tag IDs
     */
    public List<TagSolutionData> getLatestTagSolutions() {
        return tag_solutions_.toList();
    }   
    
    /**
     * Gets the current detected game pieces information.
     * Detections are updated by calling {@link #updateData()} periodically.
     *
     * @return list of recent {@link PieceDetectionData} with piece detection information
     */
    public List<PieceDetectionData> getLatestPieceDetections() {
        return piece_detections_.toList();
    }

    /**
     * Sends snapshot trigger packet for log location flagging
     *
     * @param tag_name flag name to record in log
     * @return true if packet was sent successfully, false otherwise
     */
    public boolean snapshot(String tag_name) {
        int tag_name_length = (int) MathUtil.clamp(tag_name.length(), 0, 400);
        byte[] buffer = new byte[1 + tag_name_length];
        buffer[0] = 52; // Message ID
        for (int i = 0; i < tag_name_length; i++) {
            buffer[i + 1] = (byte) Character.getNumericValue(tag_name.charAt(i));
        }

        return sendData(buffer);
    }

    /** 
     * Sends match data packet for log name syncing 
     * 
     * @return true if packet was sent successfully, false otherwise
     */
    public boolean syncMatchData() {
        String event_name = DriverStation.getEventName();
        byte[] buffer = new byte[5 + event_name.length()];
        buffer[0] = 50; // Message ID
        buffer[1] = (byte) DriverStation.getMatchNumber();
        buffer[2] = serializeMatchType();
        buffer[3] = serializeAllianceStation();
        buffer[4] = (byte) event_name.length();
        for (int i = 0; i < event_name.length(); i++) {
            buffer[i + 5] = (byte) Character.getNumericValue(event_name.charAt(i));
        }

        return sendData(buffer);
    }

    /**
     * Sends a timesync response back to the client with current server timestamps.
     * Follows the TimesyncResponseData structure for precise timing coordination.
     * 
     * @param request the parsed timesync request data
     * @return true if response was sent successfully, false otherwise
     */
    private boolean sendTimesyncResponse(TimesyncRequestData request) {
        // Create structured response with current server timestamps
        TimesyncResponse.TimesyncResponseData response = TimesyncResponse.createResponse(request);
        
        // Serialize and send response
        byte[] response_buffer = TimesyncResponse.serialize(response);
        return sendData(response_buffer);
    }

    /**
     * Sends a custom packet with specified message ID and payload data
     * 
     * @param message_id the message type identifier
     * @param payload the data payload (can be null for header-only packets)
     * @return true if packet was sent successfully, false otherwise
     */
    public boolean sendCustomPacket(byte message_id, byte[] payload) {
        byte[] buffer;
        
        if (payload == null || payload.length == 0) {
            // Header-only packet
            buffer = new byte[1];
            buffer[0] = message_id;
        } else {
            // Packet with payload
            buffer = new byte[1 + payload.length];
            buffer[0] = message_id;
            System.arraycopy(payload, 0, buffer, 1, payload.length);
        }
        
        return sendData(buffer);
    }

    /**
     * Gets the last client address that sent data to the server.
     * For multi-client tracking, use {@link #getClientAddresses()} instead.
     * 
     * @return SocketAddress of the last client that sent data, null if no client has connected
     */
    public SocketAddress getClientAddress() {
        return last_client_address_;
    }

    /**
     * Checks if the server has any active client connections.
     * 
     * @return true if at least one client is connected
     */
    public boolean hasClientConnection() {
        return !clients_.isEmpty();
    }

    /**
     * Serializes {@link DriverStation.MatchType} to byte value {None, Practice, Qualification,
     * Elimination}
     *
     * @return byte value representing match type
     */
    private byte serializeMatchType() {
        switch (DriverStation.getMatchType()) {
            case Practice:
                {
                    return 1;
                }
            case Qualification:
                {
                    return 2;
                }
            case Elimination:
                {
                    return 3;
                }
            case None:
            default:
                return 0;
        }
    }

    /**
     * Serializes DriverStation Location to byte value {Blue1, Blue2, Blue3, Red1, Red2, Red3} Will
     * return 0 if no DriverStation is Present
     *
     * @return byte value representing station location
     */
    private byte serializeAllianceStation() {
        OptionalInt optional = DriverStation.getLocation();
        if (optional.isPresent()) {
            int station = optional.getAsInt();
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                // If on Blue Alliance apply no offset {1, 2, 3}
                return (byte) station;
            } else {
                // If on Red Alliance offset by 3 {4, 5, 6}
                return (byte) (station + 3);
            }
        } else {
            // Drivers Station Not Connected.
            // This should not occur since this will only be TeleOp Init
            return 0;
        }
    }
    
    /**
     * Initializes vision simulation for generating simulated vision data.
     * This should be called in simulation mode to enable PhotonVision simulation.
     * Only works in simulation mode - does nothing on real robot.
     * 
     * @param fieldLayout the AprilTag field layout to use
     * @return the created MwVisionSimulation instance, or null if not in simulation
     */
    public MwVisionSim initializeVisionSimulation(AprilTagFieldLayout field_layout) {
        if (RobotBase.isSimulation() && visionSim == null) {
            visionSim = new MwVisionSim(field_layout);
            System.out.println("ProxyServerThread: Vision simulation initialized");
        }
        return visionSim;
    }
    
    /**
     * Updates the vision simulation with the current robot pose and generates vision data.
     * This should be called periodically with the simulated robot pose.
     * The generated vision data is automatically added to the tagSolutions queue.
     * Only works in simulation mode - does nothing on real robot.
     * 
     * @param robotPose the current simulated robot pose
     */
    public void updateVisionSimulation(Pose2d robot_pose) {
        if (!RobotBase.isSimulation() || visionSim == null) {
            return;
        }
        
        // Update the vision system with current robot pose
        visionSim.update(robot_pose);
        
        // Get generated tag solutions and add them to the queue
        List<TagSolutionData> simulated_solutions = visionSim.getTagSolutions(robot_pose);
        for (TagSolutionData solution : simulated_solutions) {
            tag_solutions_.add(solution);
        }
    }
}
