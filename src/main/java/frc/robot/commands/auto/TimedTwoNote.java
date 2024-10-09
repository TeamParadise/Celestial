// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ShootNote;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.intake.Intake;

/**
 * This command group will run a time-based (does not use PID or Pathplanning) two note center auto.
 */
public class TimedTwoNote extends SequentialCommandGroup {
  /** Creates a new TimedTwoNote. */
  public TimedTwoNote(Drive drive, Intake intake, Flywheels shooter) {
    // Add your commands in the addCommands() method below.
    // Each sequential command is seperated by a commma, and "sequential" means that each commmand
    // runs after the one before it has finished.
    addCommands(
        // The first command that runs is our ShootNote command, which will shoot our preloaded note
        // into the speaker.
        new ShootNote(
            intake, shooter), // This command ends by itself, so there is no timeout needed.

        // The second command that runs is a ParallelCommandGroup that drives backwards at 20% speed
        // as we intake.
        // This ENTIRE COMMAND GROUP has a timeout of a 4 seconds.
        DriveCommands.driveArcade(drive, () -> -0.50, () -> 0.0)
            .alongWith(new IntakeNote(intake, shooter))
            .withTimeout(2.0),
        // Notice how the "withTimeout()" is at the end of the command group? This means the entire
        // CommandGroup (so ArcadeDrive and IntakeNote) will end after 4 seconds.
        // Writing it like this does NOT result in the same thing: "new ArcadeDrive(drive, () ->
        // 0.20, () -> 0.20).WITHTIMEOUT(4.0).alongWith(new IntakeNote(intake, shooter))"
        // Writing it like that means the ArcadeDrive command ends after 4 seconds, but the
        // IntakeNote command continues on forever!
        // This means our auto will never continue on!

        // The third command that runs is just a ArcadeDrive command to drive forwards at 20% speed,
        // back to the speaker.
        DriveCommands.driveArcade(drive, () -> 0.50, () -> 0.0)
            .withTimeout(2.0), // Timeout of 2 seconds, just like the previous command group.

        // The third command that runs is another ShootNote command to (hopefully) shoot the note
        // that we (hopefully) picked up.
        new ShootNote(
            intake, shooter) // This command ends by itself, so there is no timeout needed.
        );
    // Notice the "hopefully" in the comment for the last ShootNote command? This is one of the
    // issues with time-based autos.
    // First of all, time-based autos can be inaccurate, as your batteries won't always have the
    // same voltage and sometimes you may have wheel slip, etc.
    // This is solved using PID to make sure your motors get to a set position/velocity, or
    // Pathplanning, which utilizes PID with a generated path to follow as accurately as possible.

    // Second of all, this time-based auto doesn't ensure that we get to the right position, or that
    // our flywheels are up to speed, or that we actually intake a note in the first place!
    // This means that the auto has the potential to miss sometimes.
    // But, this also means that the auto is SLOW. By utilizing commands that automatically shoot
    // once our flywheels are up to speed, or utilizing intake commands that end once
    // a game piece has been grabbed, we can make the commands only run as long as they have to, and
    // make the auto much faster
  }
}
