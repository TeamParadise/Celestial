// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;

/** Constants used for the drivetrain of the robot. */
public class DriveConstants {
  // These will need to be changed
  public static final double trackWidth = Units.inchesToMeters(21.5);
  public static final double gearRatio = 10.71;
  public static final double wheelRadius = Units.inchesToMeters(3.0);

  public static final double metersPerRotation = 2 * Math.PI * wheelRadius;

  /** Constants used on the real robot. */
  public static class RealConstants {
    public static final int leftLeaderID = 1;
    public static final int leftFollowerID = 2;
    public static final int rightLeaderID = 3;
    public static final int rightFollowerID = 4;

    // Drive PIDF values
    public static final double driveP = 0.0;
    public static final double driveI = 0.0;
    public static final double driveD = 0.0;
    public static final double driveLinearV = 0.0;
    public static final double driveLinearA = 0.0;
    public static final double driveAngularV = 0.0;
    public static final double driveAngularA = 0.0;
  }

  /** Constants used in the simulation. */
  public static class SimConstants {
    // Drive PIDF values
    public static final double driveP = 0.0;
    public static final double driveI = 0.0;
    public static final double driveD = 0.0;
    public static final double driveLinearV = 0.0;
    public static final double driveLinearA = 0.0;
    public static final double driveAngularV = 0.0;
    public static final double driveAngularA = 0.0;
  }
}
