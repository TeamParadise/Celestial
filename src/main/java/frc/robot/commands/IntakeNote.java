// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.intake.Intake;

/** This command will intake the note, and does not end by itself! */
public class IntakeNote extends ParallelCommandGroup {
  /** Creates a new IntakeNote. */
  public IntakeNote(Intake intake, Flywheels shooter) {
    // Add your commands in the addCommands() method below.
    // Each parallel command is seperated by a commma, and "parallel" means that each commmand runs
    // at the same time.
    // This means you need to make sure none of your parallel commands require the same subsystem,
    // or else your code will error.
    addCommands(
        // The first command listed is our RunIntake command, running the intake at 40% speed.
        new RunIntake(intake, 0.40),
        // The second command listed is our RunShooter command, running the shooter backwards at 20%
        // speed.
        // We do this to make sure the note doesn't fly out of the shooter as we are intaking.
        new RunShooter(shooter, -0.20) // This command runs at the same time as our intake command.
        );

    // Again, since the commands we are calling inside this command group do not end by themselves,
    // it also means that
    // the command group itself will not end by itself! If you don't end or interupt it, this
    // command will run forever.
    // You must make sure to call this from a "whileTrue()" if you are calling it from a driver
    // controller, or you must
    // add a "withTimeout()" or something similar if you are calling it from something like an
    // autonomous command.
  }
}
