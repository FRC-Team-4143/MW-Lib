package com.marswars.proxy_server;

import java.nio.ByteBuffer;

/**
 * Packet handler for time synchronization requests.
 * Provides functionality to parse incoming timesync request packets
 * containing client timing information for precision time coordination.
 */
public abstract class TimesyncRequest implements Packet {
    
    /**
     * Data structure containing time synchronization request information.
     */
    public static class TimesyncRequestData {
        /** Request identifier for matching requests to responses */
        public final int requestId;
        /** Client timestamp when request was sent (seconds) */
        public final int clientSendSeconds;
        /** Client timestamp when request was sent (nanoseconds) */
        public final int clientSendNanoseconds;
        /** Packet timestamp from header */
        public final Timestamp timestamp;

        public TimesyncRequestData(int requestId, int clientSendSeconds, 
                                 int clientSendNanoseconds, Timestamp timestamp) {
            this.requestId = requestId;
            this.clientSendSeconds = clientSendSeconds;
            this.clientSendNanoseconds = clientSendNanoseconds;
            this.timestamp = timestamp;
        }
    }
    
    // Packet type identifier
    public static final int TYPE_ID = 60;
    
    // Byte indices for timesync request data fields
    private static final int REQUEST_ID_IDX = 1;
    private static final int CLIENT_SEND_SEC_IDX = 5;
    private static final int CLIENT_SEND_NSEC_IDX = 9;
    
    /**
     * Parses timesync request packet data from a received byte buffer.
     * Extracts request ID and client timestamps from the buffer.
     * 
     * @param buffer the byte buffer containing the packet data
     * @return TimesyncRequestData object with parsed information
     */
    public static TimesyncRequestData updateData(byte[] buffer) {
        // Parse timestamp from packet header
        Timestamp timestamp = new Timestamp(
                ByteBuffer.wrap(buffer, TIME_SEC_IDX, 4).getInt(),
                ByteBuffer.wrap(buffer, TIME_NSEC_IDX, 4).getInt());
        
        // Parse request ID (default to 0 if buffer too short)
        int requestId = 0;
        if (buffer.length >= 5) {
            requestId = ByteBuffer.wrap(buffer, REQUEST_ID_IDX, 4).getInt();
        }
        
        // Parse client send timestamps (default to 0 if not available)
        int clientSendSeconds = 0;
        int clientSendNanoseconds = 0;
        if (buffer.length >= 13) {
            clientSendSeconds = ByteBuffer.wrap(buffer, CLIENT_SEND_SEC_IDX, 4).getInt();
            clientSendNanoseconds = ByteBuffer.wrap(buffer, CLIENT_SEND_NSEC_IDX, 4).getInt();
        }
        
        return new TimesyncRequestData(requestId, clientSendSeconds, clientSendNanoseconds, timestamp);
    }
}
