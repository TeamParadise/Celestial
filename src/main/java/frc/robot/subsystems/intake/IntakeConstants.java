// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static class FollowerConstants {
    public static int motorID = 6;
  }

  public static int motorID = 5;

  // PIDF Values (need to be tuned)
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;

  // Preset speeds for feeder
  public static class Presets {
    // Need to be tuned
    public static final double intake = 1000;
    public static final double feed = 1000;
  }
}
