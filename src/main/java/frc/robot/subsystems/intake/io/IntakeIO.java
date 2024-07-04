// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.io;

import org.littletonrobotics.junction.AutoLog;

/** Basic IO implementation for an Intake (with a feeder). */
public interface IntakeIO {
  /** Class to log the inputs from an Intake. */
  @AutoLog
  class IntakeIOInputs {
    // Feeder (top) wheel inputs
    public double feederPositionRotations = 0.0;
    public double feederVelocityRPM = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;

    // Intake (bottom) wheel inputs
    public double intakePositionRotations = 0.0;
    public double intakeVelocityRPM = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;
  }

  /** Updates the inputs inside an IntakeIOInputs to the latest values. */
  default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Tell the intake to run in an open-loop manner at a number of volts.
   *
   * @param feederVolts The feeder voltage to set. Value should be in between -12.0 and 12.0.
   * @param intakeVolts The intake voltage to set. Value should be in between -12.0 and 12.0.
   */
  default void setVoltage(double feederVolts, double intakeVolts) {}

  /**
   * Tell the intake to run in an open-loop manner at a specific speed.
   *
   * @param feederSpeed The feeder speed to set. Value is in between -1.0 and 1.0.
   * @param intakeSpeed The intake speed to set. Value is in between -1.0 and 1.0.
   */
  default void setSpeed(double feederSpeed, double intakeSpeed) {}

  /**
   * Tell the intake to run in a closed-loop manner at a velocity setpoint.
   *
   * @param feederRPM The left velocity to set. Value is in rotations per minute.
   * @param intakeRPM The right velocity to set. Value is in rotations per minute.
   */
  default void setVelocity(double feederRPM, double intakeRPM) {}
}
