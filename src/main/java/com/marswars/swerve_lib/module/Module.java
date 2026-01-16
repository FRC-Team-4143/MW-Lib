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

package com.marswars.swerve_lib.module;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import com.marswars.mechanisms.MechBase;
import com.marswars.swerve_lib.PhoenixOdometryThread;
import com.marswars.swerve_lib.SwerveMeasurments.ModuleMeasurement;

public abstract class Module extends MechBase {

  public enum DriveControlMode {
    OPEN_LOOP,
    CLOSED_LOOP
  }

  public enum SteerControlMode {
    CLOSED_LOOP
  }

  protected final SwerveModuleConfig config_;
  protected final int module_index_;

  protected double drive_position_rad_ = 0.0;
  protected double drive_velocity_rad_per_sec_ = 0.0;
  protected double drive_applied_volts_ = 0.0;
  protected double drive_current_amps_ = 0.0;

  protected Rotation2d turn_absolute_position_ = new Rotation2d();
  protected double turn_velocity_rad_per_sec_ = 0.0;
  protected double turn_applied_volts_ = 0.0;
  protected double turn_current_amps_ = 0.0;

  // Connection debouncers
  protected final Debouncer drive_conn_deb_ = new Debouncer(0.5);
  protected final Debouncer turn_conn_deb_ = new Debouncer(0.5);
  protected final Debouncer turn_encoder_conn_deb_ = new Debouncer(0.5);

  // alerts for disconnections
  protected final Alert drive_disconnected_alert_;
  protected final Alert turn_disconnected_alert_;
  protected final Alert turn_encoder_disconnected_alert_;

  // SIM objects
  protected final SwerveModuleSimulation simulation;

  public Module(String logging_prefix, int index, SwerveModuleConfig config, SwerveModuleSimulation simulation) {
    super(logging_prefix + "/Module" + Integer.toString(index));

    this.module_index_ = index;
    this.config_ = config;
    this.simulation = simulation;

    drive_disconnected_alert_ = new Alert(
        "Disconnected drive motor on module " + Integer.toString(config_.encoder_id) + ".",
        AlertType.kError);
    turn_disconnected_alert_ = new Alert(
        "Disconnected turn motor on module " + Integer.toString(config_.encoder_id) + ".", AlertType.kError);
    turn_encoder_disconnected_alert_ = new Alert(
        "Disconnected turn encoder on module " + Integer.toString(config_.encoder_id) + ".",
        AlertType.kError);
  }

  /**
   * Runs the module with the specified setpoint state. Mutates the state to
   * optimize it.
   */
  public void runSetpoint(
      SwerveModuleState state, DriveControlMode DriveMode, SteerControlMode SteerMode) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(turn_absolute_position_);

    // Apply setpoints
    switch (DriveMode) {
      case CLOSED_LOOP -> setDriveVelocity(state.speedMetersPerSecond / config_.wheel_radius_m);
      case OPEN_LOOP -> setDriveOpenLoop(
          state.speedMetersPerSecond / config_.speed_at_12_volts * 12.0);
    }
    switch (SteerMode) {
      case CLOSED_LOOP -> setTurnPosition(state.angle);
      // Steer open loop is for characterization only
    }
  }

  /**
   * Runs the module with the specified output while controlling to zero degrees.
   */
  public void runCharacterization(double output) {
    setDriveOpenLoop(output);
    setTurnPosition(new Rotation2d());
  }

  /** Disables all outputs to motors. */
  public void stop() {
    setDriveOpenLoop(0.0);
    setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return turn_absolute_position_;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return drive_position_rad_ * config_.wheel_radius_m;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return drive_velocity_rad_per_sec_ * config_.wheel_radius_m;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public List<ModuleMeasurement> getOdometrySamples() {
    return PhoenixOdometryThread.getInstance().getModuleSamples(module_index_);
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return drive_position_rad_;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(drive_velocity_rad_per_sec_);
  }

  /**
   * Returns the translation of the module in the robot's coordinate system.
   *
   * @return Translation2d representing the module's position in meters
   */
  public Translation2d getTranslation() {
    return new Translation2d(config_.location_x, config_.location_y);
  }

  /** Run the drive motor at the specified open loop value. */
  public abstract void setDriveOpenLoop(double output);

  /** Run the turn motor at the specified open loop value. */
  public abstract void setTurnOpenLoop(double output);

  /** Run the drive motor at the specified velocity. */
  public abstract void setDriveVelocity(double velocityRadPerSec);

  /** Run the turn motor to the specified rotation. */
  public abstract void setTurnPosition(Rotation2d rotation);

  /** Set the gains for the module drive motors to the given slot */
  public abstract void setDriveGains(int slot, SlotConfigs gains);

  /** Set the gains for the module steer motors to the given slot */
  public abstract void setSteerGains(SlotConfigs gains);
}
