// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels.io;

import org.littletonrobotics.junction.AutoLog;

/** Basic IO implementation for a set of Flywheels. */
public interface FlywheelsIO {
  /** Class to log the inputs from a set of Flywheels. */
  @AutoLog
  class FlywheelsIOInputs {
    // Top flywheel roller inputs
    public double topPositionRotations = 0.0;
    public double topVelocityRPM = 0.0;
    public double topAppliedVolts = 0.0;
    public double topCurrentAmps = 0.0;

    // Bottom flywheel roller inputs
    public double bottomPositionRotations = 0.0;
    public double bottomVelocityRPM = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double bottomCurrentAmps = 0.0;
  }
}
