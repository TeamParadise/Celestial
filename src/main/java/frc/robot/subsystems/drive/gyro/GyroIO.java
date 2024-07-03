// File originally made by: Mechanical Advantage - FRC 6328
// Copyright (c) 2024 FRC 6328 (https://github.com/Mechanical-Advantage)
// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Basic IO implementation for a Drivetrain Gyro. */
public interface GyroIO {
  /** Class used to log the inputs from a Drivetrain Gyro. */
  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yaw = new Rotation2d();
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double yawVelocityDegreesPerSecond = 0.0;
  }

  /** Updates the inputs inside a GyroIOInputs to the latest values. */
  default void updateInputs(GyroIOInputs inputs) {}

  /**
   * Tell the gyro to change the current yaw
   *
   * @param yawDegrees The number of degrees to set the yaw to. Value should be in between 0 - 360.
   */
  default void setYaw(double yawDegrees) {}
}
