// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.LoggedTunableNumber;

/** Command for automatic intake, ending when the note has been grabbed. */
public class IntakeCommand extends Command {
  // Subsystems used in this command
  private final Flywheels flywheels;
  private final Intake intake;

  // Logged and tunable values
  private final LoggedTunableNumber intakeRPM;
  private final LoggedTunableNumber flywheelRPM;

  public IntakeCommand(Flywheels flywheels, Intake intake, double flywheelRPM, double intakeRPM) {
    this.flywheels = flywheels;
    this.intake = intake;

    this.flywheelRPM = new LoggedTunableNumber("Commands/IntakeCommand/FlywheelRPM", flywheelRPM);
    this.intakeRPM = new LoggedTunableNumber("Commands/IntakeCommand/IntakeRPM", intakeRPM);

    addRequirements(this.flywheels, this.intake);
  }

  public IntakeCommand(Flywheels flywheels, Intake intake) {
    this.flywheels = flywheels;
    this.intake = intake;

    this.flywheelRPM =
        new LoggedTunableNumber(
            "Commands/IntakeCommand/FlywheelRPM", FlywheelsConstants.Presets.intake);
    this.intakeRPM =
        new LoggedTunableNumber("Commands/IntakeCommand/IntakeRPM", IntakeConstants.Presets.intake);

    addRequirements(this.flywheels, this.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Set velocity of flywheels and intake to preset value. This only needs to be set once.
    flywheels.setVelocity(flywheelRPM.get());
    intake.setVelocity(intakeRPM.get());
  }

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
