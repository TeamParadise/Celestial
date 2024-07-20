// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.io;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.subsystems.intake.IntakeConstants.*;

/** Intake IO implementation for a SPARK MAX (NEO) based intake. */
public class IntakeIOSparkMax implements IntakeIO {
  // Create basic motor objects
  private final CANSparkMax bottomIntake =
      new CANSparkMax(BottomConstants.motorID, MotorType.kBrushless);
  private final CANSparkMax topIntake = new CANSparkMax(TopConstants.motorID, MotorType.kBrushless);

  // Create color sensor object
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);

  // Get encoders for the motors
  private final RelativeEncoder bottomEncoder = bottomIntake.getEncoder();
  private final RelativeEncoder topEncoder = topIntake.getEncoder();

  // Create our PID controllers
  private final SparkPIDController bottomPID = bottomIntake.getPIDController();
  private final SparkPIDController topPID = topIntake.getPIDController();

  /** Intake IO implementation for a SPARK MAX (NEO) based intake. */
  public IntakeIOSparkMax() {
    // Reset all motor controllers to factory defaults, ensures only settings here are modified
    bottomIntake.restoreFactoryDefaults();
    topIntake.restoreFactoryDefaults();

    // Set current limits to protect motors
    bottomIntake.setSmartCurrentLimit(60);
    topIntake.setSmartCurrentLimit(60);

    // Set idle mode to coast
    bottomIntake.setIdleMode(IdleMode.kCoast);
    topIntake.setIdleMode(IdleMode.kCoast);

    // Set our PIDF values for the bottom and top PID controllers
    bottomPID.setP(BottomConstants.bottomP);
    bottomPID.setI(BottomConstants.bottomI);
    bottomPID.setD(BottomConstants.bottomD);
    bottomPID.setFF(BottomConstants.bottomF);

    topPID.setP(TopConstants.topP);
    topPID.setI(TopConstants.topI);
    topPID.setD(TopConstants.topD);
    topPID.setFF(TopConstants.topF);

    // Burn settings to the flash of the motors
    topIntake.burnFlash();
    bottomIntake.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Set inputs for bottom intake motor
    inputs.bottomPositionRotations = bottomEncoder.getPosition();
    inputs.bottomVelocityRPM = bottomEncoder.getVelocity();
    inputs.bottomAppliedVolts = bottomIntake.getAppliedOutput() * bottomIntake.getBusVoltage();
    inputs.bottomCurrentAmps = bottomIntake.getOutputCurrent();

    // Set inputs for top intake motor
    inputs.topPositionRotations = topEncoder.getPosition();
    inputs.topVelocityRPM = topEncoder.getVelocity();
    inputs.topAppliedVolts = topIntake.getAppliedOutput() * topIntake.getBusVoltage();
    inputs.topCurrentAmps = topIntake.getOutputCurrent();

    // Set inputs for proximity sensor
    inputs.proximitySensor = colorSensor.getProximity();
  }

  @Override
  public void setVoltage(double intakeVolts) {
    // Set voltage of both motors
    bottomIntake.setVoltage(intakeVolts);
    topIntake.setVoltage(intakeVolts);
  }

  @Override
  public void setSpeed(double intakeSpeed) {
    // Set speed of both motors
    bottomIntake.set(intakeSpeed);
    topIntake.set(intakeSpeed);
  }

  @Override
  public void setVelocity(double intakeRPM) {
    // I'm not sure how the REV "Motion Magic" alternative works (or if it's even still supported)
    // might want to try it though

    // Set the reference values of each PID controller
    bottomPID.setReference(intakeRPM, ControlType.kVelocity);
    topPID.setReference(intakeRPM, ControlType.kVelocity);
  }

  @Override
  public void setBottomPIDF(double bottomP, double bottomI, double bottomD, double bottomF, double bottomIz) {
    // Set the PIDF values on the bottom PID controller
    bottomPID.setP(bottomP);
    bottomPID.setI(bottomI);
    bottomPID.setD(bottomD);
    bottomPID.setFF(bottomF);
    bottomPID.setIZone(bottomIz);
  }

  @Override
  public void setTopPIDF(double topP, double topI, double topD, double topF, double topIz) {
    // Set the PIDF values on the top PID controller
    topPID.setP(topP);
    topPID.setI(topI);
    topPID.setD(topD);
    topPID.setFF(topF);
    topPID.setIZone(topIz);
  }
}
