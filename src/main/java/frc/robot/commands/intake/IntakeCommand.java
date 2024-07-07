// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

/** Command for automatic intake, ending when the note has been grabbed. */
public class IntakeCommand extends Command {
  // Subsystems used in this command
  private final Flywheels flywheels;
  private final Intake intake;

  public IntakeCommand(Flywheels flywheels, Intake intake) {
    this.flywheels = flywheels;
    this.intake = intake;

    addRequirements(this.flywheels, this.intake);
  }

  @Override
  public void initialize() {
    // Set velocity of flywheels and intake to preset value. This only needs to be set once.
    flywheels.setVelocity(FlywheelsConstants.Presets.intake);
    intake.setVelocity(IntakeConstants.Presets.intake);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    // If we think we are holding a note, we don't need to intake anymore
    return intake.isHoldingNote();
  }

  @Override
  public void end(boolean interrupted) {
    flywheels.setSpeed(0);
    intake.setSpeed(0);
  }
}
