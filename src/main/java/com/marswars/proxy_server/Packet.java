package com.marswars.proxy_server;

/**
 * Base interface for all proxy server packet types.
 * Provides common constants and utilities for packet parsing.
 */
public interface Packet {
    
    // Common packet header byte indices
    /** Packet type identifier index */
    int PACKET_ID_IDX = 0;
    /** Timestamp seconds index */
    int TIME_SEC_IDX = 1;
    /** Timestamp nanoseconds index */
    int TIME_NSEC_IDX = 5;
    
    // Legacy aliases for compatibility
    @Deprecated
    int ID_IDX = PACKET_ID_IDX;

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
