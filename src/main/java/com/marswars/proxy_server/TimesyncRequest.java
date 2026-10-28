package com.marswars.proxy_server;

import com.marswars.proxy_server.proto.CoreProto;
import com.marswars.proxy_server.proto.TimesyncProto;

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
        /** Client timestamp when request was sent */
        public final Timestamp timestamp;

        public TimesyncRequestData(int requestId, int clientSendSeconds, 
                                 int clientSendNanoseconds, Timestamp timestamp) {
            this.requestId = requestId;
            this.clientSendSeconds = clientSendSeconds;
            this.clientSendNanoseconds = clientSendNanoseconds;
            this.timestamp = timestamp;
        }
    }
    
    /**
     * Converts a received timesync request message.
     *
     * @param msg the timesync request payload of a received envelope
     * @return TimesyncRequestData object with parsed information
     */
    public static TimesyncRequestData fromProto(TimesyncProto.TimesyncRequest msg) {
        CoreProto.Timestamp clientSend = msg.getClientSend();
        return new TimesyncRequestData(msg.getReqId(), clientSend.getSec(), clientSend.getNanosec(),
                Timestamp.fromProto(clientSend));
    }
}
