package com.marswars.proxy_server;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.marswars.proxy_server.proto.ChassisProxy;

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

    /**
     * Converts a received module states message.
     *
     * @param msg the module states payload of a received envelope
     * @return ModuleStatesData object with parsed information
     */
    public static ModuleStatesData fromProto(ChassisProxy.ModuleStates msg) {
        SwerveModuleState[] states = new SwerveModuleState[msg.getModules().length()];
        for (int i = 0; i < states.length; i++) {
            ChassisProxy.SwerveModuleState module = msg.getModules().get(i);
            states[i] = new SwerveModuleState(module.getSpeedMps(), new Rotation2d(module.getAngleRad()));
        }
        return new ModuleStatesData(states, Timestamp.fromProto(msg.getStamp()));
    }
}
