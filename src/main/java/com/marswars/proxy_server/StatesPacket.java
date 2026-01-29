package com.marswars.proxy_server;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.nio.ByteBuffer;

/**
 * Packet handler for swerve drive module state data.
 * Provides functionality to parse incoming packets containing
 * the current state of all swerve modules.
 */
public abstract class StatesPacket implements Packet {

    /**
     * Data structure containing swerve module state information.
     */
    public static class ModuleStatesData {
        /** Array of swerve module states (4 modules) */
        public final SwerveModuleState[] moduleStates;
        /** Timestamp when the states were recorded */
        public final Timestamp timestamp;

        public ModuleStatesData(SwerveModuleState[] moduleStates, Timestamp timestamp) {
            this.moduleStates = moduleStates.clone();
            this.timestamp = timestamp;
        }
    }

    // Packet type identifier
    public static final int TYPE_ID = 2;
    
    // Byte indices for module state data
    private static final int MODULE_1_ANGLE_IDX = 9;
    private static final int MODULE_1_VELOCITY_IDX = 13;
    private static final int MODULE_2_ANGLE_IDX = 17;
    private static final int MODULE_2_VELOCITY_IDX = 21;
    private static final int MODULE_3_ANGLE_IDX = 25;
    private static final int MODULE_3_VELOCITY_IDX = 29;
    private static final int MODULE_4_ANGLE_IDX = 33;
    private static final int MODULE_4_VELOCITY_IDX = 37;

    private static final double STATE_RESOLUTION = 1000.0;

    /**
     * Parses module states packet data from a received byte buffer.
     * Extracts timestamp and all swerve module states from the buffer.
     * 
     * @param buffer the byte buffer containing the packet data
     * @return ModuleStatesData object with parsed information
     */
    public static ModuleStatesData updateData(byte[] buffer) {
        // Parse timestamp
        Timestamp timestamp = new Timestamp(
                ByteBuffer.wrap(buffer, TIME_SEC_IDX, 4).getInt(),
                ByteBuffer.wrap(buffer, TIME_NSEC_IDX, 4).getInt());
        
        // Parse module states
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        
        moduleStates[0] = new SwerveModuleState(
                ByteBuffer.wrap(buffer, MODULE_1_VELOCITY_IDX, 4).getInt() / STATE_RESOLUTION,
                new Rotation2d(
                        ByteBuffer.wrap(buffer, MODULE_1_ANGLE_IDX, 4).getInt() / STATE_RESOLUTION));
        
        moduleStates[1] = new SwerveModuleState(
                ByteBuffer.wrap(buffer, MODULE_2_VELOCITY_IDX, 4).getInt() / STATE_RESOLUTION,
                new Rotation2d(
                        ByteBuffer.wrap(buffer, MODULE_2_ANGLE_IDX, 4).getInt() / STATE_RESOLUTION));
        
        moduleStates[2] = new SwerveModuleState(
                ByteBuffer.wrap(buffer, MODULE_3_VELOCITY_IDX, 4).getInt() / STATE_RESOLUTION,
                new Rotation2d(
                        ByteBuffer.wrap(buffer, MODULE_3_ANGLE_IDX, 4).getInt() / STATE_RESOLUTION));
        
        moduleStates[3] = new SwerveModuleState(
                ByteBuffer.wrap(buffer, MODULE_4_VELOCITY_IDX, 4).getInt() / STATE_RESOLUTION,
                new Rotation2d(
                        ByteBuffer.wrap(buffer, MODULE_4_ANGLE_IDX, 4).getInt() / STATE_RESOLUTION));
        
        return new ModuleStatesData(moduleStates, timestamp);
    }
}
