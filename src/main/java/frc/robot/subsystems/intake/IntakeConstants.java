// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

/** Constants used for the intake of the robot. */
public class IntakeConstants {
  /** Constants for the motor that controls the bottom part of the intake. */
  public static class BottomConstants {
    public static int motorID = 5;

    // Bottom Intake Motor PIDF Values (need to be tuned)
    public static final double bottomP = 0.00015;
    public static final double bottomI = 0.000001;
    public static final double bottomD = 0.0;
    public static final double bottomF = 0.000238;
    public static final double bottomIz = 0.0;
  }

  /** Constants for the motor that controls the top part of the intake. */
  public static class TopConstants {
    public static int motorID = 6;

    // Top Intake Motor PIDF Values (need to be tuned)
    public static final double topP = 0.00015;
    public static final double topI = 0.000001;
    public static final double topD = 0.0;
    public static final double topF = 0.000238;
    public static final double topIz = 0.0;
  }

  /** Preset speeds for the intake. */
  public static class Presets {
    // Need to be tuned
    public static final double intake = 1500;
    public static final double retract = -750;
    public static final double feed = 1000;
  }
}
