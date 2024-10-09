// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.intake.Intake;

/** This command will shoot the note. */
public class ShootNote extends SequentialCommandGroup {
  /** Creates a new ShootNote. */
  public ShootNote(Intake intake, Flywheels shooter) {
    // Add your commands in the addCommands() method below.
    // Each sequential command is seperated by a commma, and "sequential" means that each commmand
    // runs after the one before it has finished.
    addCommands(
        // The first command that runs is our RunIntake command, running the intake backwards at 10%
        // speed.
        // We do this to ensure that the game piece is not touching the shooter when it starts
        // speeding up.
        new RunIntake(intake, -0.20)
            .withTimeout(
                0.25), // This "withTimeout()" means that the after 0.5 seconds, this command WILL
        // end, and the command group will move onto the next command.

        // The second command that runs is our RunShooter command, which will run the shooter at 80%
        // speed.
        // This command only runs after the RunIntake commmand ends (which in this case, is after
        // 0.25 seconds).
        // This command will also end after running for 2.5 seconds, shown by the "withTimeout()"
        // method.
        new RunShooter(shooter, 0.70)
            .alongWith( // This "alongWith()" method creates a ParallelCommandGroup! You can have
                // command groups inside of command groups.
                // Everything inside of this "alongWith()" method will run AT THE SAME TIME as the
                // RunShooter command.
                // The first commmand we run in this group is a WaitCommand, which just does nothing
                // for 1.5 seconds, it does nothing else.
                // After the WaitCommand is done (andThen, not alongWith), we use our RunIntake
                // command to intake the note at 20% speed and push it into the flywheels.
                new WaitCommand(1.5).andThen(new RunIntake(intake, 0.20)))
            .withTimeout(2.5)
        // Notice how the "withTimeout()" is at the end of the command group? This means the entire
        // CommandGroup (so RunShooter and the WaitCommand/RunIntake group) will end after 4
        // seconds.
        // Writing it like this does NOT result in the same thing: "new RunShooter(shooter,
        // 0.80).withTimeout(2.5).alongWith("
        // Writing it like that means the RunShooter command ends after 2.5 seconds, but the
        // WaitCommand/RunIntake command group continues on forever!
        // This means our shoot command will never actually end!
        );
  }
}
