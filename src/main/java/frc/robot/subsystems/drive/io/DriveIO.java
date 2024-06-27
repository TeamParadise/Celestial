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
        public double leftPositionRotations = 0.0;
        public double leftVelocityRotationsPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double[] leftCurrentAmps = new double[] {};

        public double rightPositionRotations = 0.0;
        public double rightVelocityRotationsPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double[] rightCurrentAmps = new double[] {};

        // Not sure if this robot will have a gyro yet.
        public Rotation2d gyroYaw = new Rotation2d();
    }

    /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double leftVolts, double rightVolts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {}
}
