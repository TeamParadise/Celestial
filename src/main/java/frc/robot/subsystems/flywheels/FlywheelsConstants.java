// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels;

/** Constants used for the flywheels of the robot. */
public class FlywheelsConstants {
  /** Constants for the motor that controls the bottom set of flywheels. */
  public static class BottomConstants {
    public static int motorID = 7;

    // Bottom Flywheel Motor PIDF Values (need to be tuned)
    public static final double bottomP = 0.0;
    public static final double bottomI = 0.0;
    public static final double bottomD = 0.0;
    public static final double bottomF = 0.0;
  }

  /** Constants for the motor that controls the top set of flywheels. */
  public static class TopConstants {
    public static int motorID = 8;

    // Top Flywheel Motor PIDF Values (need to be tuned)
    public static final double topP = 0.0;
    public static final double topI = 0.0;
    public static final double topD = 0.0;
    public static final double topF = 0.0;
  }

  /** Preset speeds for the flywheels. */
  public static class Presets {
    // Need to be tuned
    public static final double intake = -1000;
    public static final double retract = -500;
    public static final double shoot = 3000;
  }
}
