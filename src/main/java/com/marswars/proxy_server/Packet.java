package com.marswars.proxy_server;

import com.marswars.proxy_server.proto.ChassisProxy;

/**
 * Base interface for all proxy server packet types.
 * Packets travel as {@link ChassisProxy.Envelope} protobuf messages, defined in
 * src/main/proto/chassis_proxy/v1/chassis_proxy.proto; the packet classes convert them to and from
 * the data types the rest of the library uses.
 */
public interface Packet {

    /**
     * Timestamp class to store packet timestamp and perform timestamp operations.
     * Contains seconds and nanoseconds components for high-precision timing.
     */
    public static class Timestamp {
        public final int seconds;
        public final int nanoseconds;

        public Timestamp(int seconds, int nanoseconds) {
            this.seconds = seconds;
            this.nanoseconds = nanoseconds;
        }

        /**
         * Creates a timestamp from its protobuf form.
         *
         * @param stamp the protobuf timestamp
         * @return the equivalent timestamp
         */
        public static Timestamp fromProto(ChassisProxy.Timestamp stamp) {
            return new Timestamp(stamp.getSec(), stamp.getNanosec());
        }

        /**
         * Writes this timestamp into a protobuf timestamp.
         *
         * @param stamp the protobuf timestamp to overwrite
         * @return the protobuf timestamp, for chaining
         */
        public ChassisProxy.Timestamp toProto(ChassisProxy.Timestamp stamp) {
            return stamp.setSec(seconds).setNanosec(nanoseconds);
        }

        /**
         * Determines if this timestamp is more recent than the provided timestamp.
         *
         * @param other the timestamp to compare against
         * @return true if this timestamp is more recent, false otherwise
         */
        public boolean isMoreRecentThan(Timestamp other) {
            if (this.seconds > other.seconds) {
                return true;
            } else if (this.seconds == other.seconds && this.nanoseconds > other.nanoseconds) {
                return true;
            }
            return false;
        }
        
        /**
         * Gets the timestamp as a single double value in seconds.
         * Combines the seconds and nanoseconds into a single floating-point value.
         *
         * @return the timestamp in seconds (with nanosecond precision)
         */
        public double getSeconds() {
            return seconds + (nanoseconds / 1_000_000_000.0);
        }
        
        @Override
        public String toString() {
            return String.format("Timestamp{sec=%d, nsec=%d}", seconds, nanoseconds);
        }
        
        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            Timestamp timestamp = (Timestamp) obj;
            return seconds == timestamp.seconds && nanoseconds == timestamp.nanoseconds;
        }
        
        @Override
        public int hashCode() {
            return seconds * 31 + nanoseconds;
        }
    }
}
