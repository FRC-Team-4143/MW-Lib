package com.marswars.proxy_server;

import com.marswars.proxy_server.proto.ChassisProxy;
import us.hebi.quickbuf.InvalidProtocolBufferException;

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
    
    /**
     * Serializes a TimesyncResponseData into an envelope for transmission.
     * 
     * @param response the response data to serialize
     * @return the serialized {@link ChassisProxy.Envelope}
     */
    public static byte[] serialize(TimesyncResponseData response) {
        ChassisProxy.Envelope envelope = ChassisProxy.Envelope.newInstance()
                .setProtocolVersion(ChassisProxy.ProtocolVersion.PROTOCOL_VERSION_CURRENT.getNumber());
        ChassisProxy.TimesyncResponse msg = envelope.getMutableTimesyncResponse()
                .setReqId(response.requestId);
        msg.getMutableServerRecv()
                .setSec(response.serverReceiveSeconds)
                .setNanosec(response.serverReceiveNanoseconds);
        msg.getMutableClientSend()
                .setSec(response.clientSendSeconds)
                .setNanosec(response.clientSendNanoseconds);
        return envelope.toByteArray();
    }
    
    /**
     * Deserializes a serialized envelope carrying a timesync response.
     * 
     * @param data the serialized {@link ChassisProxy.Envelope}
     * @return TimesyncResponseData object with parsed information
     * @throws IllegalArgumentException if data is not an envelope carrying a timesync response
     */
    public static TimesyncResponseData deserialize(byte[] data) {
        ChassisProxy.Envelope envelope;
        try {
            envelope = ChassisProxy.Envelope.parseFrom(data);
        } catch (InvalidProtocolBufferException e) {
            throw new IllegalArgumentException("Invalid timesync response envelope", e);
        }
        if (!envelope.hasTimesyncResponse()) {
            throw new IllegalArgumentException("Envelope does not carry a timesync response");
        }
        
        ChassisProxy.TimesyncResponse msg = envelope.getTimesyncResponse();
        return new TimesyncResponseData(msg.getReqId(),
                msg.getServerRecv().getSec(), msg.getServerRecv().getNanosec(),
                msg.getClientSend().getSec(), msg.getClientSend().getNanosec());
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