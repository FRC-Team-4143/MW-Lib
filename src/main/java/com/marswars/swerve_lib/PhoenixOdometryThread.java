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

package com.marswars.swerve_lib;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import com.marswars.swerve_lib.SwerveMeasurements.GyroMeasurement;
import com.marswars.swerve_lib.SwerveMeasurements.ModuleMeasurement;
import com.marswars.swerve_lib.SwerveMeasurements.SwerveMeasurement;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
    private List<StatusSignal<Angle>> steer_signals_ =
            new ArrayList<>(Arrays.asList(null, null, null, null));
    private List<StatusSignal<Angle>> drive_signals_ =
            new ArrayList<>(Arrays.asList(null, null, null, null));
    private StatusSignal<Angle> gyro_signal_;

    private BaseStatusSignal[] all_signals_ = new BaseStatusSignal[0];

    private final List<Queue<ModuleMeasurement>> module_queues_;
    private final Queue<GyroMeasurement> gyro_queue_;

    // Locking for signals and odometry queue
    private final Lock signals_lock_ = new ReentrantLock();
    private final Lock odometry_lock_ = new ReentrantLock();

    private final boolean IS_CANFD;
    private final boolean IS_SIM = RobotBase.isSimulation();
    private final double ODOMETRY_FREQUENCY;
    private final String bus_name_;
    private final double wheel_circumference_m_;

    private static PhoenixOdometryThread instance = null;

    /**
     * Configures the PhoenixOdometryThread singleton.
     *
     * @param bus_name The CAN bus name the Phoenix devices are on
     * @param wheel_radius_m The wheel radius in meters
     */
    public static void configure(String bus_name, double wheel_radius_m) {
        if (instance != null) {
            DriverStation.reportWarning("PhoenixOdometryThread already configured!", false);
        }
        instance = new PhoenixOdometryThread(bus_name, wheel_radius_m);
    }

    public static PhoenixOdometryThread getInstance() {
        if (instance == null) {
            throw new IllegalStateException(
                    "PhoenixOdometryThread not yet configured. Call configure() before"
                            + " getInstance().");
        }
        return instance;
    }

    private PhoenixOdometryThread(String bus_name, double wheel_radius_m) {
        setName("PhoenixOdometryThread");
        setDaemon(true);

        bus_name_ = bus_name;
        wheel_circumference_m_ = 2.0 * Math.PI * wheel_radius_m;

        // Determine if using CAN FD bus
        if (!IS_SIM) {
            IS_CANFD = new CANBus(bus_name_).isNetworkFD();
        } else {
            IS_CANFD = true; // assume CAN FD on sim robot
        }
        ODOMETRY_FREQUENCY = IS_CANFD ? 250.0 : 100.0;

        // Queue of odometry measurements
        module_queues_ = new ArrayList<>(4);
        gyro_queue_ = new ArrayBlockingQueue<>(100);

        // create a new queue for the module
        for (int module_index = 0; module_index < 4; module_index++) {
            module_queues_.add(module_index, new ArrayBlockingQueue<>(100));
        }
    }

    @Override
    public void start() {
        if (!IS_SIM && all_signals_.length > 0) {
            super.start();
        } else {
            System.out.println(
                    "PhoenixOdometryThread not started - either in simulation or no signals"
                            + " registered.");
        }
    }

    public void registerGyro(StatusSignal<Angle> yaw_signal) {
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

    public void registerModule(
            int module_index, StatusSignal<Angle> steer_signal, StatusSignal<Angle> drive_signal) {
        if (module_index < 0 || module_index >= 4) {
            throw new IllegalArgumentException("Module index must be between 0 and 3");
        }

        signals_lock_.lock();
        try {
            // Add the signals to the all_signals array
            registerSignal(steer_signal);
            registerSignal(drive_signal);

            // also add the signals to the respective lists
            steer_signals_.set(module_index, steer_signal);
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

    private List<ModuleMeasurement> getModuleSamples(int index) {
        odometry_lock_.lock();
        List<ModuleMeasurement> samples = new ArrayList<>(module_queues_.get(index).size());
        try {
            // Empty the odometry queue into the samples list
            ModuleMeasurement sample;
            while ((sample = module_queues_.get(index).poll()) != null) {
                samples.add(sample);
            }
        } finally {
            odometry_lock_.unlock();
        }

        return samples;
    }

    public void enqueueModuleSamples(
            int index, double[] stamps, Rotation2d[] steer_positions, double[] drive_positions) {
        if (!IS_SIM) {
            DriverStation.reportWarning(
                    "Attempted to enqueue module measurement on real robot!", false);
            return;
        }

        odometry_lock_.lock();
        try {
            for (int i = 0; i < stamps.length; i++) {
                ModuleMeasurement sample = new ModuleMeasurement();
                sample.timestamp = stamps[i];
                sample.module_positions =
                        new SwerveModulePosition(
                                Units.radiansToRotations(drive_positions[i])
                                        * wheel_circumference_m_,
                                steer_positions[i]);

                // Add measurement to queue
                Queue<ModuleMeasurement> module_queue = module_queues_.get(index);
                if (!module_queue.offer(sample)) {
                    // If the queue is full, remove the oldest sample
                    module_queue.poll();
                    module_queue.offer(sample);
                }
            }
        } finally {
            odometry_lock_.unlock();
        }
    }

    private List<GyroMeasurement> getGyroSamples() {
        odometry_lock_.lock();
        List<GyroMeasurement> samples = new ArrayList<>(gyro_queue_.size());
        try {
            // Empty the odometry queue into the samples list
            GyroMeasurement sample;
            while ((sample = gyro_queue_.poll()) != null) {
                samples.add(sample);
            }
        } finally {
            odometry_lock_.unlock();
        }

        return samples;
    }

    public void enqueueGyroSamples(double[] timestamps, Rotation2d[] samples) {
        if (!IS_SIM) {
            DriverStation.reportWarning(
                    "Attempted to enqueue gyro measurement on real robot!", false);
            return;
        }

        odometry_lock_.lock();
        try {
            for (int i = 0; i < samples.length; i++) {
                GyroMeasurement sample = new GyroMeasurement();
                sample.timestamp = timestamps[i];
                sample.gyro_yaw = samples[i];

                // Add measurement to queue
                if (!gyro_queue_.offer(sample)) {
                    // If the queue is full, remove the oldest sample
                    gyro_queue_.poll();
                    gyro_queue_.offer(sample);
                }
            }
        } finally {
            odometry_lock_.unlock();
        }
    }

    public List<SwerveMeasurement> getSwerveSamples() {
        List<SwerveMeasurement> swerveSamples = new ArrayList<>();
        odometry_lock_.lock();

        try {
            // Empty the odometry queue into the samples list
            SwerveMeasurement sample;
            int num_samples = gyro_queue_.size();
            for (int i = 0; i <= 3; i++) {
                num_samples = Math.min(num_samples, module_queues_.get(i).size());
            }

            for (int i = 0; i < num_samples; i++) {
                sample = new SwerveMeasurement();
                GyroMeasurement gyroMeasurement = gyro_queue_.poll();
                sample.timestamp = gyroMeasurement.timestamp;
                sample.gyro_yaw = gyroMeasurement.gyro_yaw;

                SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
                for (int j = 0; j < 4; j++) {
                    ModuleMeasurement moduleMeasurement = module_queues_.get(j).poll();
                    modulePositions[j] = moduleMeasurement.module_positions;
                }
                sample.module_positions = modulePositions;

                swerveSamples.add(sample);
            }
        } finally {
            odometry_lock_.unlock();
        }

        return swerveSamples;
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
                    if (all_signals_.length > 0) BaseStatusSignal.refreshAll(all_signals_);
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

                // Build a new module measurement for each module
                for (int module_index = 0; module_index < 4; module_index++) {
                    ModuleMeasurement measurement = new ModuleMeasurement();
                    measurement.timestamp = timestamp;
                    measurement.module_positions =
                            new SwerveModulePosition(
                                    drive_signals_.get(module_index).getValue().in(Rotations)
                                            * wheel_circumference_m_,
                                    Rotation2d.fromRadians(
                                            steer_signals_
                                                    .get(module_index)
                                                    .getValue()
                                                    .in(Radians)));

                    // Add measurement to queue
                    Queue<ModuleMeasurement> module_queue = module_queues_.get(module_index);
                    if (!module_queue.offer(measurement)) {
                        // If the queue is full, remove the oldest sample
                        module_queue.poll();
                        module_queue.offer(measurement);
                    }
                }

                // Build a new gyro measurement
                if (gyro_signal_ != null) {
                    GyroMeasurement gyro_measurement = new GyroMeasurement();
                    gyro_measurement.timestamp = timestamp;
                    gyro_measurement.gyro_yaw =
                            Rotation2d.fromRadians(gyro_signal_.getValue().in(Radians));

                    // Add measurement to queue
                    if (!gyro_queue_.offer(gyro_measurement)) {
                        // If the queue is full, remove the oldest sample
                        gyro_queue_.poll();
                        gyro_queue_.offer(gyro_measurement);
                    }
                }

            } finally {
                odometry_lock_.unlock();
            }
        }
    }
}
