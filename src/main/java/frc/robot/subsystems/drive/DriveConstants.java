// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

/** Add your docs here. */
public class DriveConstants {
    public static final int leftLeaderID = 1;
    public static final int leftFollowerID = 2;
    public static final int rightLeaderID = 3;
    public static final int rightFollowerID = 4;
 
    // This will need to be changed
    public static final double gearRatio = 5.0;

    // PIDF Values (need to be tuned)
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kF = 0.0;
}
