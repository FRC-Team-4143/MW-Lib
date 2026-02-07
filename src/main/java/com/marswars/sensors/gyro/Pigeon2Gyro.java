// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.marswars.sensors.gyro;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import com.marswars.swerve_lib.PhoenixOdometryThread;

/** IO implementation for Pigeon 2. */
public class Pigeon2Gyro extends Gyro {

    // real pigeon members
    private Pigeon2 pigeon;
    private StatusSignal<Angle> yaw;
    private StatusSignal<Angle> pitch;
    private StatusSignal<Angle> roll;
    private StatusSignal<AngularVelocity> yawVelocity;

    public Pigeon2Gyro(
            String logging_prefix, int id, String can_bus_name) {
        super(logging_prefix);
        
        if (!IS_SIM) {
            // Create Pigeon
            pigeon = new Pigeon2(id, can_bus_name);
            yaw = pigeon.getYaw();
            pitch = pigeon.getPitch();
            roll = pigeon.getRoll();
            yawVelocity = pigeon.getAngularVelocityZWorld();

            // Configure Pigeon
            pigeon.getConfigurator().apply(new Pigeon2Configuration());
            pigeon.getConfigurator().setYaw(0.0);
            yaw.setUpdateFrequency(new CANBus(can_bus_name).isNetworkFD() ? 250.0 : 100.0);
            pitch.setUpdateFrequency(50.0);
            roll.setUpdateFrequency(50.0);
            yawVelocity.setUpdateFrequency(50.0);
            pigeon.optimizeBusUtilization();

            // Register to the odometry thread
            PhoenixOdometryThread.getInstance().registerGyro(yaw);
        }
        // In simulation, no hardware initialization needed
    }

    @Override
    public void readGyro() {
        if (IS_SIM) {
            // In simulation, report as disconnected so odometry-based yaw is used
            connected = false;
            yawPosition = new Rotation2d();
            pitchPosition = new Rotation2d();
            rollPosition = new Rotation2d();
            yawVelocityRadPerSec = 0.0;
        } else {
            connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
            yawPosition = Rotation2d.fromRadians(MathUtil.angleModulus(yaw.getValue().in(Radians)));
            yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
            pitchPosition = Rotation2d.fromRadians(MathUtil.angleModulus(pitch.getValue().in(Radians)));
            rollPosition = Rotation2d.fromRadians(MathUtil.angleModulus(roll.getValue().in(Radians)));
        }
    }

    @Override
    public void setYaw(Rotation2d yaw) {
        if (!IS_SIM) {
            pigeon.setYaw(yaw.getDegrees());
        }
        // In simulation, do nothing - odometry handles yaw
    }
}
