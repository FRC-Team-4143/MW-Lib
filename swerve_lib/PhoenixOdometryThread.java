// Copyright 2021-2025 FRC 6328
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

package frc.mw_lib.swerve_lib;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements
 * to a set of queues.
 *
 * <p>
 * This version is intended for Phoenix 6 devices on both the RIO and CANivore
 * buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more
 * consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between
 * devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {

  // Basic structure to hold an odometry measurement
  public class OdometryMeasurement {
    public double timestamp;
    public double[] turn_positions_rad;
    public double[] drive_positions_rad;
    public double gyro_yaw_rad;

    public OdometryMeasurement(double timestamp, Angle value) {
      this.timestamp = timestamp;
      this.turn_positions_rad = new double[4];
      this.drive_positions_rad = new double[4];
    }
  }

  private List<StatusSignal<Double>> turn_signals_ = new ArrayList<>(4);
  private List<StatusSignal<Double>> drive_signals_ = new ArrayList<>(4);
  private StatusSignal<Double> gyro_signal_;

  private BaseStatusSignal[] all_signals_ = new BaseStatusSignal[0];

  private final Queue<OdometryMeasurement> odometry_queue_;

  // Locking for signals and odometry queue
  private final Lock signals_lock_ = new ReentrantLock();
  private final Lock odometry_lock_ = new ReentrantLock();

  private final boolean IS_CANFD;
  private final double ODOMETRY_FREQUENCY;

  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance(boolean is_canfd) {
    if (instance == null) {
      instance = new PhoenixOdometryThread(is_canfd);
    }
    return instance;
  }

  private PhoenixOdometryThread(boolean is_canfd) {
    setName("PhoenixOdometryThread");
    setDaemon(true);

    // Determine if using CAN FD bus
    IS_CANFD = is_canfd;
    ODOMETRY_FREQUENCY = IS_CANFD ? 250.0 : 100.0;

    // Queue of odometry measurements
    odometry_queue_ = new ArrayBlockingQueue<>(100);
  }

  @Override
  public void start() {
    if (RobotBase.isReal() && all_signals_.length > 0) {
      super.start();
    }
  }

  public void registerGyro(StatusSignal<Double> yaw_signal) {
    signals_lock_.lock();
    try {
      // Add the signal to the all_signals array
      registerSignal(yaw_signal);

      // also add the gyro signal to the yaw_signals list
      gyro_signal_ = yaw_signal;
    } finally {
      signals_lock_.unlock();
    }
  }

  public void registerModule(int module_index, StatusSignal<Double> turn_signal, StatusSignal<Double> drive_signal) {
    if (module_index < 0 || module_index >= 4) {
      throw new IllegalArgumentException("Module index must be between 0 and 3");
    }

    signals_lock_.lock();
    try {
      // Add the signals to the all_signals array
      registerSignal(turn_signal);
      registerSignal(drive_signal);

      // also add the signals to the respective lists
      turn_signals_.set(module_index, turn_signal);
      drive_signals_.set(module_index, drive_signal);
    } finally {
      signals_lock_.unlock();
    }
  }

  protected void registerSignal(BaseStatusSignal signal) {
    // Add the signal to the all_signals array
    BaseStatusSignal[] new_signals = new BaseStatusSignal[all_signals_.length + 1];
    System.arraycopy(all_signals_, 0, new_signals, 0, all_signals_.length);
    new_signals[all_signals_.length] = signal;
    all_signals_ = new_signals;
  }

  public List<OdometryMeasurement> getOdometrySamples() {
    odometry_lock_.lock();
    List<OdometryMeasurement> samples = new ArrayList<>(odometry_queue_.size());
    try {
      // Empty the odometry queue into the samples list
      OdometryMeasurement sample;
      while ((sample = odometry_queue_.poll()) != null) {
        samples.add(sample);
      }
    } finally {
      odometry_lock_.unlock();
    }

    return samples;
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      signals_lock_.lock();
      try {
        if (IS_CANFD && all_signals_.length > 0) {
          BaseStatusSignal.waitForAll(2.0 / ODOMETRY_FREQUENCY, all_signals_);
        } else {
          // "waitForAll" does not support blocking on multiple signals with a bus
          // that is not CAN FD, regardless of Pro licensing. No reasoning for this
          // behavior is provided by the documentation.
          Thread.sleep((long) (1000.0 / ODOMETRY_FREQUENCY));
          if (all_signals_.length > 0)
            BaseStatusSignal.refreshAll(all_signals_);
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        signals_lock_.unlock();
      }

      // Save new data to queues
      odometry_lock_.lock();
      try {
        // Sample timestamp is current FPGA time minus average CAN latency
        // Default timestamps from Phoenix are NOT compatible with
        // FPGA timestamps, this solution is imperfect but close
        double timestamp = RobotController.getFPGATime() / 1e6;
        double totalLatency = 0.0;
        for (BaseStatusSignal signal : all_signals_) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (all_signals_.length > 0) {
          timestamp -= totalLatency / all_signals_.length;
        }

        // Add new samples to queues

        OdometryMeasurement measurement = new OdometryMeasurement(timestamp, null);
        measurement.gyro_yaw_rad = Math.toRadians(gyro_signal_.getValue());

        for (int i = 0; i < 4; i++) {
          measurement.turn_positions_rad[i] = Math.toRadians(turn_signals_.get(i).getValue());
          measurement.drive_positions_rad[i] = Math.toRadians(drive_signals_.get(i).getValue());
        }

        // Add measurement to queue
        if (!odometry_queue_.offer(measurement)) {
          // If the queue is full, remove the oldest sample
          odometry_queue_.poll();
          odometry_queue_.offer(measurement);
        }

      } finally {
        odometry_lock_.unlock();
      }
    }
  }
}
