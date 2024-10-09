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

    // Bottom Flywheel Motor PIDF Values
    public static final double bottomP = 0.000059987;
    public static final double bottomI = 0.0;
    public static final double bottomD = 0.0;
    public static final double bottomS = 0.34849;
    public static final double bottomV = 0.0020871;
    public static final double bottomA = 0.00022852;
    public static final double bottomIz = 0.0;

    // Maximum acceleration value for Motion Magic Velocity or Smart Velocity
    public static final double bottomMaxAccel = 9000.0;
  }

  /** Constants for the motor that controls the top set of flywheels. */
  public static class TopConstants {
    public static int motorID = 8;

    // Top Flywheel Motor PIDF Values
    public static final double topP = 0.00006;
    public static final double topI = 0.0;
    public static final double topD = 0.0;
    public static final double topS = 0.40977;
    public static final double topV = 0.0020871;
    public static final double topA = 0.00018081;
    public static final double topIz = 0.0;

    // Maximum acceleration value for Motion Magic Velocity or Smart Velocity
    public static final double topMaxAccel = 9000.0;
  }

  /** Preset speeds for the flywheels. */
  public static class Presets {
    public static final double intake = -1000;
    public static final double retract = -500;
    public static final double speaker = 3250;
    // The amp value is the speed of the bottom flywheel, the top flywheel is slower
    public static final double amp = 1200;
    public static final double pass = 5500;
    public static final double toss = 500;
  }
}
