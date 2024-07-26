// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.io;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.RealConstants;

/** Drive IO implementation for a SPARK MAX (NEO) based drivetrain. */
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

  // Get encoders for the motors
  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

  // Create our PID controllers
  private final SparkPIDController leftPID = leftLeader.getPIDController();
  private final SparkPIDController rightPID = rightLeader.getPIDController();

  /** Drive IO implementation for a SPARK MAX (NEO) based drivetrain. */
  public DriveIOSparkMax() {
    // Reset all motor controllers to factory defaults, ensures only settings here are modified
    leftLeader.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightLeader.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    // Set current limits to protect motors
    leftLeader.setSmartCurrentLimit(60);
    leftFollower.setSmartCurrentLimit(60);
    rightLeader.setSmartCurrentLimit(60);
    rightFollower.setSmartCurrentLimit(60);

    // Invert left side of the drivetrain
    leftLeader.setInverted(true);

    // Set follow motors to follow
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    // Set our PIDF values for both the left and right PID controllers
    leftPID.setP(RealConstants.leftP);
    leftPID.setI(RealConstants.leftI);
    leftPID.setD(RealConstants.leftD);
    leftPID.setFF(RealConstants.leftF);

    rightPID.setP(RealConstants.rightP);
    rightPID.setI(RealConstants.rightI);
    rightPID.setD(RealConstants.rightD);
    rightPID.setFF(RealConstants.rightF);

    // Burn settings to the flash of all the motors
    leftLeader.burnFlash();
    leftFollower.burnFlash();
    rightLeader.burnFlash();
    rightFollower.burnFlash();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    // Set inputs for left drive side
    inputs.leftPositionRotations = leftEncoder.getPosition() / DriveConstants.gearRatio;
    inputs.leftVelocityRPM = leftEncoder.getVelocity() / DriveConstants.gearRatio;
    inputs.leftAppliedVolts = leftLeader.getAppliedOutput() * leftLeader.getBusVoltage();
    inputs.leftCurrentAmps =
        new double[] {leftLeader.getOutputCurrent(), leftFollower.getOutputCurrent()};

    // Set inputs for right drive side
    inputs.rightPositionRotations = rightEncoder.getPosition() / DriveConstants.gearRatio;
    inputs.rightVelocityRPM = rightEncoder.getVelocity() / DriveConstants.gearRatio;
    inputs.rightAppliedVolts = rightLeader.getAppliedOutput() * rightLeader.getBusVoltage();
    inputs.rightCurrentAmps =
        new double[] {rightLeader.getOutputCurrent(), rightFollower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    // Set voltage of both motors
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }

  @Override
  public void setSpeed(double leftSpeed, double rightSpeed) {
    // Set speed of both motors
    leftLeader.set(leftSpeed);
    rightLeader.set(rightSpeed);
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

  @Override
  public void setBrakeMode(boolean enable) {
    leftLeader.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    rightLeader.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setLeftPIDF(double leftP, double leftI, double leftD, double leftF) {
    // Set the PIDF values on the left PID controller
    leftPID.setP(leftP);
    leftPID.setI(leftI);
    leftPID.setD(leftD);
    leftPID.setFF(leftF);
  }

  @Override
  public void setRightPIDF(double rightP, double rightI, double rightD, double rightF) {
    // Set the PIDF values on the right PID controller
    rightPID.setP(rightP);
    rightPID.setI(rightI);
    rightPID.setD(rightD);
    rightPID.setFF(rightF);
  }
}
