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

/** Command for automatic intake, ending when the note has been grabbed */
public class IntakeCommand extends Command {

  private final Flywheels flywheels;
  private final Intake intake;

  public IntakeCommand(Flywheels flywheels, Intake intake) {
    this.flywheels = flywheels;
    this.intake = intake;

    addRequirements(this.flywheels, this.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    flywheels.setVelocity(FlywheelsConstants.Presets.intake);
    intake.setVelocity(IntakeConstants.Presets.intake);
  }

  /**
   * Returns whether this command has finished. Once a command finishes -- indicated by this method
   * returning true -- the scheduler will call its {@link #end(boolean)} method.
   *
   * <p>Returning false will result in the command never ending automatically. It may still be
   * cancelled manually or interrupted by another command. Hard coding this command to always return
   * true will result in the command executing once and finishing immediately. It is recommended to
   * use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an
   * operation.
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished() {
    // Add code here to check if the note has been grabbed using the current of motors, needs to be
    // tested before it can be finalized.
    return false;
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally --
   * that is it is called when {@link #isFinished()} returns true -- or when it is
   * interrupted/canceled. This is where you may want to wrap up loose ends, like shutting off a
   * motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    flywheels.setVelocity(0);
    intake.setVelocity(0);
  }
}
