// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.io;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import frc.robot.subsystems.drive.DriveConstants.RealConstants;

public class DriveIOSparkMax implements DriveIO {
  // Create all of our basic motor objects to be used for the drivetrain
  private final CANSparkMax leftLeader =
      new CANSparkMax(RealConstants.leftLeaderID, MotorType.kBrushless);
  private final CANSparkMax leftFollower =
      new CANSparkMax(RealConstants.leftFollowerID, MotorType.kBrushless);
  private final CANSparkMax rightLeader =
      new CANSparkMax(RealConstants.rightLeaderID, MotorType.kBrushless);
  private final CANSparkMax rightFollower =
      new CANSparkMax(RealConstants.rightFollowerID, MotorType.kBrushless);

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
    leftPID.setP(RealConstants.kP);
    leftPID.setI(RealConstants.kI);
    leftPID.setD(RealConstants.kD);
    leftPID.setFF(RealConstants.kV);

    rightPID.setP(RealConstants.kP);
    rightPID.setI(RealConstants.kI);
    rightPID.setD(RealConstants.kD);
    rightPID.setFF(RealConstants.kV);
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
    leftPID.setReference(
        leftMetersPerSec / DriveConstants.metersPerRotation * 60, ControlType.kVelocity);
    rightPID.setReference(
        rightMetersPerSec / DriveConstants.metersPerRotation * 60, ControlType.kVelocity);
  }
}
