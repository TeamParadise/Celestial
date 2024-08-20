// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels.io;

import org.littletonrobotics.junction.AutoLog;

/** Basic IO implementation for a set of Flywheels. */
public interface FlywheelsIO {
  /** Class to log the inputs from a set of Flywheels. */
  @AutoLog
  class FlywheelsIOInputs {
    // Bottom flywheel roller inputs
    public double bottomPositionRotations = 0.0;
    public double bottomVelocityRPM = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double bottomCurrentAmps = 0.0;

    // Top flywheel roller inputs
    public double topPositionRotations = 0.0;
    public double topVelocityRPM = 0.0;
    public double topAppliedVolts = 0.0;
    public double topCurrentAmps = 0.0;
  }

  /** Updates the inputs inside an FlywheelsIOInputs to the latest values. */
  default void updateInputs(FlywheelsIOInputs inputs) {}

  /**
   * Tell the flywheels to run in an open-loop manner at a number of volts.
   *
   * @param flywheelVolts The flywheel voltage to set. Value should be in between -12.0 and 12.0.
   */
  default void setVoltage(double flywheelVolts) {}

  /**
   * Tell the flywheels to run in an open-loop manner at a specific speed.
   *
   * @param flywheelSpeed The flywheel speed to set. Value is in between -1.0 and 1.0.
   */
  default void setSpeed(double flywheelSpeed) {}

  /**
   * Tell the flywheels to run in a closed-loop manner at a velocity setpoint.
   *
   * @param flywheelRPM The flywheel velocity to set. Value is in rotations per minute.
   */
  default void setVelocity(double flywheelRPM) {}

  /**
   * Tell the flywheels to run in a closed-loop manner at a velocity setpoint.
   *
   * @param bottomRPM The bottom flywheel velocity to set. Value is in rotations per minute.
   * @param topRPM The top flywheel velocity to set. Value is in rotations per minute.
   */
  default void setVelocity(double bottomRPM, double topRPM) {}

  /** Change the PIDF values on the bottom set of flywheels. */
  default void setBottomPIDF(
      double bottomP, double bottomI, double bottomD, double bottomS, double bottomV, double bottomA, double bottomIz) {}

  /** Change the PIDF values on the top set of flywheels. */
  default void setTopPIDF(double topP, double topI, double topD, double topS, double topV, double topA, double topIz) {}
}
