// File originally made by: Mechanical Advantage - FRC 6328
// Copyright (c) 2024 FRC 6328 (https://github.com/Mechanical-Advantage)
// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.SimConstants;

/** Drive IO implementation for a simulated differential drivetrain */
public class DriveIOSim implements DriveIO {
  // Built-in WPILib drivetrain simulation
  private final DifferentialDrivetrainSim sim;

  // Create closed loop boolean so that we know when to run closed loop
  private boolean closedLoop = false;

  // Store simulated drivetrain voltage values
  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  // Create PID controller for both sides (need to create some sort of sim pid values)
  private final PIDController leftPID =
      new PIDController(SimConstants.driveP, SimConstants.driveI, SimConstants.driveD);
  private final PIDController rightPID =
      new PIDController(SimConstants.driveP, SimConstants.driveI, SimConstants.driveD);

  // Create feedforward controller to calculate feedforward values
  private SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(0, SimConstants.driveLinearV);

  // "Constructor" class, run when the class is first initialized
  public DriveIOSim(
      KitbotMotor simulatedMotors, KitbotGearing gearRatio, KitbotWheelSize wheelSize) {
    // I hate these Kitbot classes. There must be a better way to sync the real gear ratio and the
    // sim gear ratio. I'll probably try to create a custom DifferentialDrivetrainSim class
    // eventually.
    // Setup drivetrain simulation
    sim = DifferentialDrivetrainSim.createKitbotSim(simulatedMotors, gearRatio, wheelSize, null);
  }

  // In simulation, calling "updateInputs" also actually updates the simulation
  public void updateInputs(DriveIOInputs inputs) {
    // If we are in closed-loop control, run PID and feedforward calculations
    if (closedLoop) {
      // Calculate the PID and feedforward values for the left and right side, and clamp them to a
      // minimum of -12 volts and a maximum of 12 volts.
      leftAppliedVolts =
          MathUtil.clamp(
              leftPID.calculate(
                      sim.getLeftVelocityMetersPerSecond() / DriveConstants.metersPerRotation)
                  + feedforward.calculate(
                      sim.getLeftVelocityMetersPerSecond() / DriveConstants.metersPerRotation),
              -12.0,
              12.0);
      rightAppliedVolts =
          MathUtil.clamp(
              rightPID.calculate(
                      sim.getRightVelocityMetersPerSecond() / DriveConstants.metersPerRotation)
                  + feedforward.calculate(
                      sim.getRightVelocityMetersPerSecond() / DriveConstants.metersPerRotation),
              -12.0,
              12.0);

      // Send these values to the simulation
      sim.setInputs(leftAppliedVolts, rightAppliedVolts);
    }

    // Update simulation with current values
    sim.update(Constants.loopPeriodSecs);

    // Set inputs for left drive side
    inputs.leftPositionRotations = sim.getLeftPositionMeters() / DriveConstants.metersPerRotation;
    inputs.leftVelocityRPM =
        sim.getLeftVelocityMetersPerSecond() * 60 / DriveConstants.metersPerRotation;
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] {sim.getLeftCurrentDrawAmps()};

    // Set inputs for right drive side
    inputs.rightPositionRotations = sim.getRightPositionMeters() / DriveConstants.metersPerRotation;
    inputs.rightVelocityRPM =
        sim.getRightVelocityMetersPerSecond() * 60 / DriveConstants.metersPerRotation;
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] {sim.getRightCurrentDrawAmps()};

    // Set inputs for simulated heading
    inputs.simulatedYaw = sim.getHeading();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    // No longer running in closed loop, set closed loop to false
    closedLoop = false;

    // Ensure both values are in between -12.0 and 12.0
    leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);

    // Send values to the simulation
    sim.setInputs(leftAppliedVolts, rightAppliedVolts);
  }

  @Override
  public void setSpeed(double leftSpeed, double rightSpeed) {
    // Convert speed values to voltage and set the voltage
    setVoltage(leftSpeed * 12.0, rightSpeed * 12.0);
  }

  @Override
  public void setVelocity(double leftMetersPerSec, double rightMetersPerSec) {
    // Now running in closed loop, set closed loop to true
    closedLoop = true;

    // Set the setpoint of the PID loops
    leftPID.setSetpoint(leftMetersPerSec);
    rightPID.setSetpoint(rightMetersPerSec);
  }

  @Override
  public void setLeftPID(double leftP, double leftI, double leftD) {
    // Set the PID values on the left PID controller
    leftPID.setPID(leftP, leftI, leftD);
  }

  @Override
  public void setRightPID(double rightP, double rightI, double rightD) {
    // Set the PID values on the left PID controller
    rightPID.setPID(rightP, rightI, rightD);
  }
}
