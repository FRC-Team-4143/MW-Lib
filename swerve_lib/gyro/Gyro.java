package frc.mw_lib.swerve_lib.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Gyro {

  private final GyroIO io;
  private final GyroIO.GyroIOInputs inputs = new GyroIO.GyroIOInputs();
  private final Alert gyroDisconnectedAlert;

  public Gyro(GyroIO io) {
    this.io = io;
    gyroDisconnectedAlert =
        new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    gyroDisconnectedAlert.set(!inputs.connected);
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public Rotation2d getYawPosition() {
    return inputs.yawPosition;
  }

  public double getYawVelocityRadPerSec() {
    return inputs.yawVelocityRadPerSec;
  }

  public double[] getOdometryYawTimestamps() {
    return inputs.odometryYawTimestamps;
  }

  public Rotation2d[] getOdometryYawPositions() {
    return inputs.odometryYawPositions;
  }
}
