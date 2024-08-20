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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.flywheels.FlywheelsConstants.*;

/** Flywheels IO implementation for a SPARK MAX (NEO) based set of flywheels. */
public class FlywheelsIOSparkMax implements FlywheelsIO {
  // Create basic motor objects
  private final CANSparkMax bottomFlywheel =
      new CANSparkMax(BottomConstants.motorID, MotorType.kBrushless);
  private final CANSparkMax topFlywheel =
      new CANSparkMax(TopConstants.motorID, MotorType.kBrushless);

  // Get encoder for the motor
  private final RelativeEncoder bottomEncoder = bottomFlywheel.getEncoder();
  private final RelativeEncoder topEncoder = topFlywheel.getEncoder();

  // Create our PID controllers
  private final SparkPIDController bottomPID = bottomFlywheel.getPIDController();
  private final SparkPIDController topPID = topFlywheel.getPIDController();

  // Create our feedforward controllers
  private SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(BottomConstants.bottomS, BottomConstants.bottomV, BottomConstants.bottomA);
  private SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(TopConstants.topS, TopConstants.topV, TopConstants.topA);

  /** Flywheels IO implementation for a SPARK MAX (NEO) based set of flywheels. */
  public FlywheelsIOSparkMax() {
    // Reset all motor controllers to factory defaults, ensures only settings here are modified
    bottomFlywheel.restoreFactoryDefaults();
    topFlywheel.restoreFactoryDefaults();

    // Set current limits to protect motors
    bottomFlywheel.setSmartCurrentLimit(60);
    topFlywheel.setSmartCurrentLimit(60);

    // Set idle mode to coast
    bottomFlywheel.setIdleMode(IdleMode.kCoast);
    topFlywheel.setIdleMode(IdleMode.kCoast);

    // Invert flywheel motors
    bottomFlywheel.setInverted(true);
    topFlywheel.setInverted(true);

    // Set our PID values for the bottom and top PID controllers
    bottomPID.setP(BottomConstants.bottomP);
    bottomPID.setI(BottomConstants.bottomI);
    bottomPID.setD(BottomConstants.bottomD);
    topPID.setP(TopConstants.topP);
    topPID.setI(TopConstants.topI);
    topPID.setD(TopConstants.topD);

    // Burn settings to the flash of the motors
    bottomFlywheel.burnFlash();
    topFlywheel.burnFlash();
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    // Set inputs for bottom intake motor
    inputs.bottomPositionRotations = bottomEncoder.getPosition();
    inputs.bottomVelocityRPM = bottomEncoder.getVelocity();
    inputs.bottomAppliedVolts = bottomFlywheel.getAppliedOutput() * bottomFlywheel.getBusVoltage();
    inputs.bottomCurrentAmps = bottomFlywheel.getOutputCurrent();

    // Set inputs for top intake motor
    inputs.topPositionRotations = topEncoder.getPosition();
    inputs.topVelocityRPM = topEncoder.getVelocity();
    inputs.topAppliedVolts = topFlywheel.getAppliedOutput() * topFlywheel.getBusVoltage();
    inputs.topCurrentAmps = topFlywheel.getOutputCurrent();
  }

  @Override
  public void setVoltage(double flywheelVolts) {
    // Set voltage of both motors
    bottomFlywheel.setVoltage(flywheelVolts);
    topFlywheel.setVoltage(flywheelVolts);
  }

  @Override
  public void setSpeed(double flywheelSpeed) {
    // Set speed of both motors
    bottomFlywheel.set(flywheelSpeed);
    topFlywheel.set(flywheelSpeed);
  }

  @Override
  public void setVelocity(double flywheelRPM) {
    // Set the reference values of each PID controller, adding our own feedforward values
    bottomPID.setReference(flywheelRPM, ControlType.kVelocity, 0, bottomFeedforward.calculate(bottomEncoder.getVelocity()), SparkPIDController.ArbFFUnits.kVoltage);
    topPID.setReference(flywheelRPM, ControlType.kVelocity, 0, topFeedforward.calculate(topEncoder.getVelocity()), SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setVelocity(double bottomRPM, double topRPM) {
    bottomPID.setReference(bottomRPM, ControlType.kVelocity, 0, bottomFeedforward.calculate(bottomEncoder.getVelocity()), SparkPIDController.ArbFFUnits.kVoltage);
    topPID.setReference(topRPM, ControlType.kVelocity, 0, topFeedforward.calculate(topEncoder.getVelocity()), SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setBottomPIDF(
      double bottomP, double bottomI, double bottomD, double bottomS, double bottomV, double bottomA, double bottomIz) {
    // Set the PID values on the bottom PID controller
    bottomPID.setP(bottomP);
    bottomPID.setI(bottomI);
    bottomPID.setD(bottomD);
    bottomPID.setIZone(bottomIz);

    // Set the feedforward values by creating a new bottom SimpleMotorFeedforward
    bottomFeedforward = new SimpleMotorFeedforward(bottomS, bottomV, bottomA);
  }

  @Override
  public void setTopPIDF(double topP, double topI, double topD, double topS, double topV, double topA, double topIz) {
    // Set the PID values on the top PID controller
    topPID.setP(topP);
    topPID.setI(topI);
    topPID.setD(topD);
    topPID.setIZone(topIz);

    // Set the feedforward values by creating a new top SimpleMotorFeedforward
    topFeedforward = new SimpleMotorFeedforward(topS, topV, topA);
  }
}
