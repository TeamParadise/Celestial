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

/** Command for manual flywheel/intake control (a command that doesn't have an end condition). */
public class ManualCommand extends Command {
  // Subsystems used in this command
  private final Flywheels flywheels;
  private final Intake intake;

  // Logged and tunable values
  private final LoggedTunableNumber intakeRPM;
  private final LoggedTunableNumber flywheelRPM;

  public ManualCommand(Flywheels flywheels, Intake intake, double flywheelRPM, double intakeRPM) {
    this.flywheels = flywheels;
    this.intake = intake;

    this.flywheelRPM = new LoggedTunableNumber("Commands/ManualCommand/FlywheelRPM", flywheelRPM);
    this.intakeRPM = new LoggedTunableNumber("Commands/ManualCommand/IntakeRPM", intakeRPM);

    addRequirements(this.flywheels, this.intake);
  }

  public ManualCommand(Flywheels flywheels, Intake intake) {
    this.flywheels = flywheels;
    this.intake = intake;

    this.flywheelRPM =
        new LoggedTunableNumber(
            "Commands/ManualCommand/FlywheelRPM", FlywheelsConstants.Presets.intake);
    this.intakeRPM =
        new LoggedTunableNumber("Commands/ManualCommand/IntakeRPM", IntakeConstants.Presets.intake);

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
    // This command doesn't have an end condition. Make sure to end it with something like a
    // "whileTrue" button press!
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    flywheels.setSpeed(0);
    intake.setSpeed(0);
  }
}
