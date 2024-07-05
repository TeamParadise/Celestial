// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.io;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.robot.subsystems.intake.IntakeConstants;

/** Intake IO implementation for a SPARK MAX (NEO) based intake. */
public class IntakeIOSparkMax implements IntakeIO {
  // Create basic motor objects (leader is the top motor, follower is the bottom)
  private final CANSparkMax intakeLeader =
      new CANSparkMax(IntakeConstants.motorID, MotorType.kBrushless);
  private final CANSparkMax intakeFollower =
      new CANSparkMax(IntakeConstants.FollowerConstants.motorID, MotorType.kBrushless);

  // Get encoder for the motor
  private final RelativeEncoder intakeEncoder = intakeLeader.getEncoder();

  // Create our PID controller
  private final SparkPIDController intakePID = intakeLeader.getPIDController();

  // "Constructor" class, run when the class if first initialized
  public IntakeIOSparkMax() {
    // Reset all motor controllers to factory defaults, ensures only settings here are modified
    intakeLeader.restoreFactoryDefaults();
    intakeFollower.restoreFactoryDefaults();

    // Set current limits to protect motors
    intakeLeader.setSmartCurrentLimit(60);
    intakeFollower.setSmartCurrentLimit(60);

    // Set idle mode to coast
    intakeLeader.setIdleMode(IdleMode.kCoast);
    intakeFollower.setIdleMode(IdleMode.kCoast);

    // Set our PIDF values for the intake PID controller
    intakePID.setP(IntakeConstants.kP);
    intakePID.setI(IntakeConstants.kI);
    intakePID.setD(IntakeConstants.kD);
    intakePID.setFF(IntakeConstants.kV);

    // Burn settings to the flash of the motors
    intakeLeader.burnFlash();
    intakeFollower.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Set inputs for intake
    inputs.intakePositionRotations = intakeEncoder.getPosition();
    inputs.intakeVelocityRPM = intakeEncoder.getVelocity();
    inputs.intakeAppliedVolts = intakeLeader.getAppliedOutput() * intakeLeader.getBusVoltage();
    inputs.intakeCurrentAmps =
        new double[] {intakeLeader.getOutputCurrent(), intakeFollower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double intakeVolts) {
    // Set voltage of intake
    intakeLeader.setVoltage(intakeVolts);
  }

  @Override
  public void setSpeed(double intakeSpeed) {
    // Set speed of both motors
    intakeLeader.set(intakeSpeed);
  }

  @Override
  public void setVelocity(double intakeRPM) {
    // I'm not sure how the REV "Motion Magic" alternative works (or if it's even still supported)
    // might want to try it though

    // Set the reference values of each PID controller
    intakePID.setReference(intakeRPM, ControlType.kVelocity);
  }
}
