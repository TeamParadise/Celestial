// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheels.Flywheels;

public class RunShooter extends Command {
  // Create basic subsystems objects to be called
  private final Flywheels shooter;

  // Create the speed object that will be changed when we create this command
  private final double shooterSpeed;

  /** Creates a new RunShooter. */
  public RunShooter(Flywheels shooter, double shooterSpeed) {
    // Set the "shooter" of this command equal to the shooter argument that was passed in. This
    // makes the variable global (inside of this command).
    this.shooter = shooter;

    // Set the speed of this command equal to the speed that was passed in, making it global (in
    // this command).
    this.shooterSpeed = shooterSpeed;

    // Add the shooter subsystem as a requirement of this command, prevents from running any other
    // commands that require the shooter at the same time as this one.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Call the shooter subsystem and set the speed
    shooter.setSpeed(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Once this command ends, set the shooter speed to zero.
    shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
