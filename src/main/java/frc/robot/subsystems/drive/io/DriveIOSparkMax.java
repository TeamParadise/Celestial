// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.io;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.subsystems.drive.DriveConstants;

public class DriveIOSparkMax {
    // Create all of our basic motor objects to be used for the drivetrain
    private final CANSparkMax leftLeader = new CANSparkMax(DriveConstants.leftLeaderID, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(DriveConstants.leftFollowerID, MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(DriveConstants.rightLeaderID, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(DriveConstants.rightFollowerID, MotorType.kBrushless);
    
    // "Constructor" class, run when the class is first initialized
    public DriveIOSparkMax() {
        // Reset all motor controllers to factory defaults, ensures only settings here are modified
        leftLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        // Invert right side of the drivetrain
        rightLeader.setInverted(true);

        // Set follow motors to follow
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
    }
}
