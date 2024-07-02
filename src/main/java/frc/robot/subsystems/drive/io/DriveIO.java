// File originally made by: Mechanical Advantage - FRC 6328
// Copyright (c) 2024 FRC 6328 (https://github.com/Mechanical-Advantage)
// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.io;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    // Left drive inputs
    public double leftPositionRotations = 0.0;
    public double leftVelocityRotationsPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {};

    // Right drive inputs
    public double rightPositionRotations = 0.0;
    public double rightVelocityRotationsPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {};

    // Not sure if this robot will have a gyro yet.
    public Rotation2d gyroYaw = new Rotation2d();
  }

  /** Updates the inputs inside a DriveIOInputs to the latest values. */
  default void updateInputs(DriveIOInputs inputs) {}

  /**
   * Tell the drivetrain to run in an open-loop manner at a number of volts.
   *
   * @param leftVolts The left voltage to set. Value should be in between -12.0 and 12.0.
   * @param rightVolts The right voltage to set. Value should be in between -12.0 and 12.0.
   */
  default void setVoltage(double leftVolts, double rightVolts) {}

  /**
   * Tell the drivetrain to run in an open-loop manner at a specific speed.
   *
   * @param leftSpeed The left speed to set. Value is in between -1.0 and 1.0.
   * @param rightSpeed The right speed to set. Value is in between -1.0 and 1.0.
   */
  default void setSpeed(double leftSpeed, double rightSpeed) {}

  /**
   * Tell the drivetrain to run in a closed-loop manner at a velocity setpoint.
   *
   * @param leftMetersPerSec The left velocity to set. Value is in meters per second.
   * @param rightMetersPerSec The right velocity to set. Value is in meter sper second.
   */
  default void setVelocity(double leftMetersPerSec, double rightMetersPerSec) {}

  /** Enable or disable brake mode on the drivetrain. */
  default void setBrakeMode(boolean enable) {}
}
