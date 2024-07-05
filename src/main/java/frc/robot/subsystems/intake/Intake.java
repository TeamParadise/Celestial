// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Class for controlling a basic single-motor Intake */
public class Intake extends SubsystemBase {
  // IntakeIO objects
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Class for controlling a basic single-motor Intake */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update and log inputs from the intake
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /** Tell the intake to run in an open-loop manner at a certain number of volts. */
  public void driveVolts(double intakeVolts) {
    Logger.recordOutput("Intake/ClosedLoop/Active", false);

    io.setVoltage(intakeVolts);
  }

  /** Tell the intake to run in a closed-loop manner with velocity control. */
  public void driveVelocity(double intakeRPM) {
    Logger.recordOutput("Intake/ClosedLoop/Active", true);
    Logger.recordOutput("Intake/VelocitySetpointRPM", intakeRPM);

    io.setVelocity(intakeRPM);
  }

  /** Returns the velocity of the intake in rotations per minute (RPM). */
  @AutoLogOutput
  public double getIntakeVelocity() {
    return inputs.intakeVelocityRPM;
  }
}
