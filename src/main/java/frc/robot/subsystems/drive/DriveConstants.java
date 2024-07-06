// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;

public class DriveConstants {
  // These will need to be changed
  public static final double trackWidth = Units.inchesToMeters(26.0);
  public static final double gearRatio = 8.45;
  public static final double wheelRadius = Units.inchesToMeters(3.0);

  public static final double metersPerRotation = 2 * Math.PI * wheelRadius;

  public static class RealConstants {
    public static final int leftLeaderID = 1;
    public static final int leftFollowerID = 2;
    public static final int rightLeaderID = 3;
    public static final int rightFollowerID = 4;

    // Left PIDF Values (need to be tuned)
    public static final double leftP = 0.0;
    public static final double leftI = 0.0;
    public static final double leftD = 0.0;
    public static final double leftF = 0.0;

    // Right PIDF Values (need to be tuned)
    public static final double rightP = 0.0;
    public static final double rightI = 0.0;
    public static final double rightD = 0.0;
    public static final double rightF = 0.0;
  }

  public static class SimConstants {
    // Left PID Values (need to be tuned)
    public static final double leftP = 0.0;
    public static final double leftI = 0.0;
    public static final double leftD = 0.0;

    // Right PID Values (need to be tuned)
    public static final double rightP = 0.0;
    public static final double rightI = 0.0;
    public static final double rightD = 0.0;

    // Shared feedforward value
    public static final double feedforward = 0.0;
  }
}
