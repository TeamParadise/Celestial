// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake.io;

import org.littletonrobotics.junction.AutoLog;

/** Basic IO implementation for a dual-motor Intake. */
public interface IntakeIO {
  /** Class to log the inputs from an Intake. */
  @AutoLog
  class IntakeIOInputs {
    // Bottom motor inputs
    public double bottomPositionRotations = 0.0;
    public double bottomVelocityRPM = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double bottomCurrentAmps = 0.0;

    // Top motor inputs
    public double topPositionRotations = 0.0;
    public double topVelocityRPM = 0.0;
    public double topAppliedVolts = 0.0;
    public double topCurrentAmps = 0.0;

    // Proximity sensor inputs
    public int proximitySensor = 0;
  }

  /** Updates the inputs inside an IntakeIOInputs to the latest values. */
  default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Tell the intake to run in an open-loop manner at a number of volts.
   *
   * @param intakeVolts The intake voltage to set. Value should be in between -12.0 and 12.0.
   */
  default void setVoltage(double intakeVolts) {}

  /**
   * Tell the intake to run in an open-loop manner at a specific speed.
   *
   * @param intakeSpeed The intake speed to set. Value is in between -1.0 and 1.0.
   */
  default void setSpeed(double intakeSpeed) {}

  /**
   * Tell the intake to run in a closed-loop manner at a velocity setpoint.
   *
   * @param intakeRPM The intake velocity to set. Value is in rotations per minute.
   */
  default void setVelocity(double intakeRPM) {}

  /** Change the PIDF values on the bottom motor of the intake. */
  default void setBottomPIDF(double bottomP, double bottomI, double bottomD, double bottomF) {}

  /** Change the PIDF values on the top motor of the intake. */
  default void setTopPIDF(double topP, double topI, double topD, double topF) {}
}
