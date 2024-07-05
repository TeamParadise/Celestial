// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.flywheels.io.FlywheelsIO;
import frc.robot.subsystems.flywheels.io.FlywheelsIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Class for controlling a set of Flywheels. */
public class Flywheels extends SubsystemBase {
  // FlywheeslIO objects
  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  /** Class for controlling a set of Flywheels. */
  public Flywheels(FlywheelsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update and log inputs from the intake
    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);
  }

  /** Tell the flywheels to run in an open-loop manner at a certain number of volts. */
  public void setVolts(double flywheelVolts) {
    Logger.recordOutput("Flywheels/ClosedLoop/Active", false);

    io.setVoltage(flywheelVolts);
  }

  /** Tell the flywheels to run in a closed-loop manner with velocity control. */
  public void setVelocity(double flywheelRPM) {
    Logger.recordOutput("Flywheels/ClosedLoop/Active", true);
    Logger.recordOutput("Flywheels/VelocitySetpointRPM", flywheelRPM);

    io.setVelocity(flywheelRPM);
  }

  /** Returns the velocity of the flywheels in rotations per minute (RPM). */
  @AutoLogOutput
  public double getFlywheelVelocity() {
    return inputs.topVelocityRPM;
  }
}
