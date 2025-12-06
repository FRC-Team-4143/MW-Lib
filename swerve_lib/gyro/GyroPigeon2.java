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

package frc.mw_lib.swerve_lib.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.mw_lib.mechanisms.PhoenixOdometryThread;
import frc.mw_lib.util.PhoenixUtil;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Queue;

import org.ironmaple.simulation.drivesims.GyroSimulation;

/** IO implementation for Pigeon 2. */
public class GyroPigeon2 extends Gyro {

  // real pigeon members
  private Pigeon2 pigeon;
  private StatusSignal<Angle> yaw;
  private Queue<Double> yawPositionQueue;
  private Queue<Double> yawTimestampQueue;
  private StatusSignal<AngularVelocity> yawVelocity;

  // sim pigeon members
  private GyroSimulation gyroSimulation;

  public GyroPigeon2(int id, String can_bus_name, GyroSimulation gyroSimulation) {
    if (!IS_SIM) {
      // Create Pigeon
      pigeon = new Pigeon2(id, can_bus_name);
      yaw = pigeon.getYaw();
      yawVelocity = pigeon.getAngularVelocityZWorld();

      // Configure Pigeon
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
      pigeon.getConfigurator().setYaw(0.0);
      yaw.setUpdateFrequency(new CANBus(can_bus_name).isNetworkFD() ? 250.0 : 100.0);
      yawVelocity.setUpdateFrequency(50.0);
      pigeon.optimizeBusUtilization();
      yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
    } else {
      this.gyroSimulation = gyroSimulation;
    }
  }

  @Override
  public void readGyro() {
    if (!IS_SIM) {
      connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
      yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
      yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

      odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
      odometryYawPositions = yawPositionQueue.stream()
          .map((Double value) -> Rotation2d.fromDegrees(value))
          .toArray(Rotation2d[]::new);
      yawTimestampQueue.clear();
      yawPositionQueue.clear();
    } else {
      connected = true;
      yawPosition = gyroSimulation.getGyroReading();
      yawVelocityRadPerSec = Units
          .degreesToRadians(gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

      odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
      odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    }
  }
}
