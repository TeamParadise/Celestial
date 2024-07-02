// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.io;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.RealConstants;

public class DriveIOSparkMax implements DriveIO {
    // Create all of our basic motor objects to be used for the drivetrain
    private final CANSparkMax leftLeader = new CANSparkMax(DriveConstants.leftLeaderID, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(DriveConstants.leftFollowerID, MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(DriveConstants.rightLeaderID, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(DriveConstants.rightFollowerID, MotorType.kBrushless);

    // Create our PID controllers
    private final SparkPIDController leftPID = leftLeader.getPIDController();
    private final SparkPIDController rightPID = rightLeader.getPIDController();
    
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

        // Set our PIDF values for both the left and right PID controllers
        leftPID.setP(DriveConstants.kP);
        leftPID.setI(DriveConstants.kI);
        leftPID.setD(DriveConstants.kD);
        leftPID.setFF(DriveConstants.kF);

        rightPID.setP(DriveConstants.kP);
        rightPID.setI(DriveConstants.kI);
        rightPID.setD(DriveConstants.kD);
        rightPID.setFF(DriveConstants.kF);
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
    }

    @Override
    public void setVelocity(double leftMetersPerSec, double rightMetersPerSec) {
        // I'm not sure how the REV "Motion Magic" alternative works (or if it's even still supported)
        // might want to try it though

        // Set the reference values of each PID controller 
        leftPID.setReference(leftMetersPerSec / DriveConstants.metersPerRotation * 60, ControlType.kVelocity);
        rightPID.setReference(rightMetersPerSec / DriveConstants.metersPerRotation * 60, ControlType.kVelocity);
    }
}