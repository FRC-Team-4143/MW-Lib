package com.marswars.proxy_server;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

import com.marswars.data_structures.ConcurrentFifoQueue;
import com.marswars.proxy_server.OdomPacket.OdometryData;
import com.marswars.proxy_server.PieceDetectionPacket.PieceDetectionData;
import com.marswars.proxy_server.StatesPacket.ModuleStatesData;
import com.marswars.proxy_server.TagSolutionPacket.TagSolutionData;
import com.marswars.proxy_server.TimesyncRequest.TimesyncRequestData;
import com.marswars.swerve_lib.PhoenixOdometryThread;
import com.marswars.util.NumUtil;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalInt;

/**
 * Proxy server thread for handling UDP communication with external systems.
 * Receives and processes various packet types including odometry, module states,
 * AprilTag solutions, piece detections, and timesync requests.
 * Provides bidirectional communication capabilities on a single UDP port.
 */
public class ProxyServerThread extends Thread {

    // Data packet queues for storing received information
    private final ConcurrentFifoQueue<OdometryData> odometryReadings = new ConcurrentFifoQueue<>(20);
    private final ConcurrentFifoQueue<ModuleStatesData> moduleStatesReadings = new ConcurrentFifoQueue<>(20);
    private final ConcurrentFifoQueue<TagSolutionData> tagSolutions = new ConcurrentFifoQueue<>(20);
    private final ConcurrentFifoQueue<PieceDetectionData> pieceDetections = new ConcurrentFifoQueue<>(20);

    // Socket Config
    private final int PORT = 5809; // local port to bind server
    private DatagramSocket socket_ = null;
    private final int TIMEOUT = 1; // Server receive blocking timeout
    
    // Client connection info for sending data back
    private SocketAddress client_address_ = null;

    // Singleton instance
    private static ProxyServerThread instance_ = null;

    /**
     * Gets the singleton instance of the ProxyServerThread.
     *
     * @return the ProxyServerThread instance
     * @throws IllegalStateException if the instance has not been configured yet
     */
    public static ProxyServerThread getInstance() {
        if (instance_ == null) {
            throw new IllegalStateException(
                    "ProxyServerThread not yet configured. Call configure() before"
                            + " getInstance().");
        } else {
            instance_ = new ProxyServerThread();
            instance_.configureServer();
        }
        return instance_;
    }

    @Override
    public void start(){
        if(!(RobotBase.isSimulation())){
            configureServer();
            super.start();
        }
        else{
            System.out.println("ProxyServerThread not started - in simulation");
        }
    }

    @Override
    public void run(){
        updateData();
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
            
            // Store client address for sending data back
            client_address_ = packet.getSocketAddress();

            // Determine packet type from first byte of buffer
            switch ((int) buffer[Packet.PACKET_ID_IDX]) {
                // Odometry Packet Type
                case OdomPacket.TYPE_ID: // const uint8_t msg_id{ 30u };
                    odometryReadings.add(OdomPacket.updateData(buffer));
                    break;
                // Module States Packet Type
                case StatesPacket.TYPE_ID: // const uint8_t msg_id{ 2u };
                    moduleStatesReadings.add(StatesPacket.updateData(buffer));
                    break;
                // AprilTag Solution Packet Type
                case TagSolutionPacket.TYPE_ID: // const uint8_t msg_id{ 15u };
                    tagSolutions.add(TagSolutionPacket.updateData(buffer));
                    break;
                // Piece Detection Packet Type
                case PieceDetectionPacket.TYPE_ID: // const uint8_t msg_id{ 10u };
                    pieceDetections.add(PieceDetectionPacket.updateData(buffer));
                    break;
                // Timesync Request Packet Type
                case TimesyncRequest.TYPE_ID: // const uint8_t msg_id{ 60u };
                    // Parse request and send response
                    TimesyncRequestData timesyncRequest = TimesyncRequest.updateData(buffer);
                    sendTimesyncResponse(timesyncRequest);
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
     * Sends data back to the last connected client on the same port
     * 
     * @param buffer the byte array to send
     * @return true if data was sent successfully, false if there was an error
     */
    public boolean sendData(byte[] buffer) {
        if (socket_ == null || !socket_.isBound()) {
            System.err.println("Socket not configured. Call configureServer() first.");
            return false;
        }
        
        if (client_address_ == null) {
            System.err.println("No client address available. Must receive data first.");
            return false;
        }
        
        try {
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length, client_address_);
            socket_.send(packet);
            return true;
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
    }

    /**
     * Gets the current robot pose, velocity, and variance data from odometry readings.
     * Data is updated by calling {@link #updateData()} periodically.
     *
     * @return list of recent {@link OdometryData} from chassis proxy
     */
    public List<OdometryData> getLatestOdometryReadings() {
        return odometryReadings.toList();
    }

    /**
     * Gets the current robot module states information.
     * States are updated by calling {@link #updateData()} periodically.
     *
     * @return list of recent {@link ModuleStatesData} containing swerve module information
     */
    public List<ModuleStatesData> getLatestModuleStatesReadings() {
        return moduleStatesReadings.toList();
    }

    /**
     * Gets the current robot module states as individual SwerveModuleState arrays.
     * This is a convenience method that extracts the module states arrays from the data objects.
     * 
     * @return list of SwerveModuleState arrays (for backward compatibility)
     */
    public List<SwerveModuleState[]> getLatestModuleStatesArrays() {
        List<SwerveModuleState[]> moduleStatesArrays = new ArrayList<>();
        for (ModuleStatesData statesData : moduleStatesReadings.toList()) {
            moduleStatesArrays.add(statesData.moduleStates);
        }
        return moduleStatesArrays;
    }

    /**
     * Gets the current AprilTag solution information.
     * Solutions are updated by calling {@link #updateData()} periodically.
     *
     * @return list of recent {@link TagSolutionData} with pose estimates and detected tag IDs
     */
    public List<TagSolutionData> getLatestTagSolutions() {
        return tagSolutions.toList();
    }   
    
    /**
     * Gets the current detected game pieces information.
     * Detections are updated by calling {@link #updateData()} periodically.
     *
     * @return list of recent {@link PieceDetectionData} with piece detection information
     */
    public List<PieceDetectionData> getLatestPieceDetections() {
        return pieceDetections.toList();
    }

    /**
     * Sends snapshot trigger packet for log location flagging
     *
     * @param tag_name flag name to record in log
     * @return true if packet was sent successfully, false otherwise
     */
    public boolean snapshot(String tag_name) {
        int tag_name_length = (int) NumUtil.clamp(tag_name.length(), 400);
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
        byte[] responseBuffer = TimesyncResponse.serialize(response);
        return sendData(responseBuffer);
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
     * Gets the current client address that the server is communicating with
     * 
     * @return SocketAddress of the last client that sent data, null if no client has connected
     */
    public SocketAddress getClientAddress() {
        return client_address_;
    }

    /**
     * Checks if the server has an active client connection
     * 
     * @return true if there is a client address available for sending data
     */
    public boolean hasClientConnection() {
        return client_address_ != null;
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
}
