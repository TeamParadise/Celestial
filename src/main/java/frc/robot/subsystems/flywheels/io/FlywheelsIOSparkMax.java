// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels.io;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.robot.subsystems.flywheels.FlywheelsConstants;
import frc.robot.subsystems.flywheels.FlywheelsConstants.*;

/** Flywheels IO implementation for a SPARK MAX (NEO) based intake. */
public class FlywheelsIOSparkMax implements FlywheelsIO {
  // Create basic motor objects
  private final CANSparkMax topFlywheel =
      new CANSparkMax(TopConstants.motorID, MotorType.kBrushless);
  private final CANSparkMax bottomFlywheel =
      new CANSparkMax(BottomConstants.motorID, MotorType.kBrushless);

  // Get encoder for the motor
  private final RelativeEncoder topEncoder = topFlywheel.getEncoder();
  private final RelativeEncoder bottomEncoder = bottomFlywheel.getEncoder();

  // Create our PID controllers
  private final SparkPIDController flywheelPID = topFlywheel.getPIDController();

  // "Constructor" class, run when the class if first initialized
  public FlywheelsIOSparkMax() {
    // Reset all motor controllers to factory defaults, ensures only settings here are modified
    topFlywheel.restoreFactoryDefaults();
    bottomFlywheel.restoreFactoryDefaults();

    // Set current limits to protect motors
    topFlywheel.setSmartCurrentLimit(60);
    bottomFlywheel.setSmartCurrentLimit(60);

    // Set idle mode to coast
    topFlywheel.setIdleMode(IdleMode.kCoast);
    bottomFlywheel.setIdleMode(IdleMode.kCoast);

    // Set the bottom flywheel to be a follower motor
    // This might have to be changed later if the PID values for each motor need to be seperate
    bottomFlywheel.follow(topFlywheel, true);

    // Set our PIDF values for the intake PID controller
    flywheelPID.setP(FlywheelsConstants.kP);
    flywheelPID.setI(FlywheelsConstants.kI);
    flywheelPID.setD(FlywheelsConstants.kD);
    flywheelPID.setFF(FlywheelsConstants.kV);

    // Burn settings to the flash of the motors
    topFlywheel.burnFlash();
    bottomFlywheel.burnFlash();
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    // Set inputs for the top flywheel motor
    inputs.topPositionRotations = topEncoder.getPosition();
    inputs.topVelocityRPM = topEncoder.getVelocity();
    inputs.topAppliedVolts = topFlywheel.getAppliedOutput() * topFlywheel.getBusVoltage();
    inputs.topCurrentAmps = topFlywheel.getOutputCurrent();

    // Set inputs for the bottom flywheel motor
    inputs.bottomPositionRotations = bottomEncoder.getPosition();
    inputs.bottomVelocityRPM = bottomEncoder.getVelocity();
    inputs.bottomAppliedVolts = bottomFlywheel.getAppliedOutput() * bottomFlywheel.getBusVoltage();
    inputs.bottomCurrentAmps = bottomFlywheel.getOutputCurrent();
  }

  @Override
  public void setVoltage(double flywheelVolts) {
    // Set voltage of intake
    topFlywheel.setVoltage(flywheelVolts);
  }

  @Override
  public void setSpeed(double flywheelSpeed) {
    // Set speed of both motors
    topFlywheel.set(flywheelSpeed);
  }

  @Override
  public void setVelocity(double flywheelRPM) {
    // I'm not sure how the REV "Motion Magic" alternative works (or if it's even still supported)
    // might want to try it though

    // Set the reference values of each PID controller
    flywheelPID.setReference(flywheelRPM, ControlType.kVelocity);
  }
}
