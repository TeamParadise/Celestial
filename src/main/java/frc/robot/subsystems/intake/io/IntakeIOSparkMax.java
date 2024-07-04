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
import frc.robot.subsystems.intake.IntakeConstants.FeederConstants;

/** Intake IO implementation for a SPARK MAX (NEO) based intake. */
public class IntakeIOSparkMax implements IntakeIO {
  // Create basic motor objects
  private final CANSparkMax feederMotor =
      new CANSparkMax(FeederConstants.motorID, MotorType.kBrushless);
  private final CANSparkMax intakeMotor =
      new CANSparkMax(IntakeConstants.motorID, MotorType.kBrushless);

  // Get encoder for the motors
  private final RelativeEncoder feederEncoder = feederMotor.getEncoder();
  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  // Create our PID controllers
  private final SparkPIDController feederPID = feederMotor.getPIDController();
  private final SparkPIDController intakePID = intakeMotor.getPIDController();

  // "Constructor" class, run when the class if first initialized
  public IntakeIOSparkMax() {
    // Reset all motor controllers to factory defaults, ensures only settings here are modified
    feederMotor.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();

    // Set current limits to protect motors
    feederMotor.setSmartCurrentLimit(60);
    intakeMotor.setSmartCurrentLimit(60);

    // Set our PIDF values for both the feeder and intake PID controllers
    feederPID.setP(FeederConstants.kP);
    feederPID.setI(FeederConstants.kI);
    feederPID.setD(FeederConstants.kD);
    feederPID.setFF(FeederConstants.kV);

    intakePID.setP(IntakeConstants.kP);
    intakePID.setI(IntakeConstants.kI);
    intakePID.setD(IntakeConstants.kD);
    intakePID.setFF(IntakeConstants.kV);

    // Set idle mode to coast
    intakeMotor.setIdleMode(IdleMode.kCoast);
    feederMotor.setIdleMode(IdleMode.kCoast);

    // Burn settings to the flash of the motors
    feederMotor.burnFlash();
    intakeMotor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Set inputs for feeder motor
    inputs.feederPositionRotations = feederEncoder.getPosition();
    inputs.feederVelocityRPM = feederEncoder.getVelocity();
    inputs.feederAppliedVolts = feederMotor.getAppliedOutput() * feederMotor.getBusVoltage();
    inputs.feederCurrentAmps = feederMotor.getOutputCurrent();

    // Set inputs for right drive side
    inputs.intakePositionRotations = intakeEncoder.getPosition();
    inputs.intakeVelocityRPM = intakeEncoder.getVelocity();
    inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double feederVolts, double intakeVolts) {
    // Set voltage of both motors
    feederMotor.setVoltage(feederVolts);
    intakeMotor.setVoltage(intakeVolts);
  }

  @Override
  public void setSpeed(double feederSpeed, double intakeSpeed) {
    // Set speed of both motors
    feederMotor.set(feederSpeed);
    intakeMotor.set(intakeSpeed);
  }

  @Override
  public void setVelocity(double feederRPM, double intakeRPM) {
    // I'm not sure how the REV "Motion Magic" alternative works (or if it's even still supported)
    // might want to try it though

    // Set the reference values of each PID controller
    feederPID.setReference(feederRPM, ControlType.kVelocity);
    intakePID.setReference(intakeRPM, ControlType.kVelocity);
  }
}
