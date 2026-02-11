package com.marswars.proxy_server;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Packet handler for time synchronization responses.
 * Provides functionality to create and serialize timesync response packets
 * containing server timing information for precision time coordination.
 */
public abstract class TimesyncResponse implements Packet {
    
    /**
     * Data structure containing time synchronization response information.
     */
    public static class TimesyncResponseData {
        /** Request identifier echoed from the original request */
        public final int requestId;
        /** Server timestamp when request was received (seconds) */
        public final int serverReceiveSeconds;
        /** Server timestamp when request was received (nanoseconds) */
        public final int serverReceiveNanoseconds;
        /** Client timestamp when response was sent (seconds) */
        public final int clientSendSeconds;
        /** Client timestamp when response was sent (nanoseconds) */
        public final int clientSendNanoseconds;

        public TimesyncResponseData(int requestId, int serverReceiveSeconds, 
                                  int serverReceiveNanoseconds, int clientSendSeconds, 
                                  int clientSendNanoseconds) {
            this.requestId = requestId;
            this.serverReceiveSeconds = serverReceiveSeconds;
            this.serverReceiveNanoseconds = serverReceiveNanoseconds;
            this.clientSendSeconds = clientSendSeconds;
            this.clientSendNanoseconds = clientSendNanoseconds;
        }
    }
    
    // Packet type identifier
    public static final int TYPE_ID = 61;
    
    /**
     * Serializes a TimesyncResponseData into a byte array for transmission.
     * 
     * @param response the response data to serialize
     * @return byte array representing the serialized response packet
     */
    public static byte[] serialize(TimesyncResponseData response) {
        byte[] buffer = new byte[21]; // 1 + 4*5 = 21 bytes
        
        ByteBuffer bb = ByteBuffer.wrap(buffer);
        bb.put((byte) TYPE_ID);                           // msg_id
        bb.putInt(response.requestId);                    // req_id
        bb.putInt(response.serverReceiveSeconds);         // server_recv_sec
        bb.putInt(response.serverReceiveNanoseconds);     // server_recv_nanosec
        bb.putInt(response.clientSendSeconds);            // server_send_sec
        bb.putInt(response.clientSendNanoseconds);        // server_send_nanosec
        
        return buffer;
    }
    
    /**
     * Deserializes a byte array into a TimesyncResponseData object.
     * 
     * @param data the byte array to deserialize
     * @return TimesyncResponseData object with parsed information
     * @throws IllegalArgumentException if data length is invalid
     */
    public static TimesyncResponseData deserialize(byte[] data) {
        if (data.length < 21) {
            throw new IllegalArgumentException("Invalid timesync response data length: " + data.length);
        }
        
        ByteBuffer bb = ByteBuffer.wrap(data);
        bb.get(); // skip message ID
        
        int requestId = bb.getInt();
        int serverReceiveSeconds = bb.getInt();
        int serverReceiveNanoseconds = bb.getInt();
        int serverSendSeconds = bb.getInt();
        int serverSendNanoseconds = bb.getInt();
        
        return new TimesyncResponseData(requestId, serverReceiveSeconds, serverReceiveNanoseconds,
                                      serverSendSeconds, serverSendNanoseconds);
    }
    
    /**
     * Creates a timesync response for the given request with current server timestamps.
     * 
     * @param request the original timesync request data
     * @return TimesyncResponseData with current server timestamps
     */
    public static TimesyncResponseData createResponse(TimesyncRequest.TimesyncRequestData request) {
        // Get current system time for server timestamps
        long currentTime = RobotController.getFPGATime();
        int serverTimeSeconds = (int) (currentTime / 1e6);
        int serverTimeNanoseconds = (int) ((currentTime % 1e6) * 1e3);
        
        // Use same timestamp for both receive and send for simplicity
        return new TimesyncResponseData(
            request.requestId,
            serverTimeSeconds,     // server_recv_sec
            serverTimeNanoseconds, // server_recv_nanosec
            request.clientSendSeconds,     // server_send_sec (same as recv for simplicity)
            request.clientSendNanoseconds  // server_send_nanosec (same as recv for simplicity)
        );
    }
}