// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.flywheels;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.intake.Intake;

public class ShootCommand extends Command {

  private final Flywheels flywheels;
  private final Intake intake;

  public ShootCommand(Flywheels flywheels, Intake intake) {
    this.flywheels = flywheels;
    this.intake = intake;

    addRequirements(this.flywheels, this.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
