// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RunIntake extends Command {
  // Create basic subsystems objects to be called
  private final Intake intake;

  // Create the speed object that will be changed when we create this command
  private final double intakeSpeed;

  /** Creates a new RunIntake. */
  public RunIntake(Intake intake, double intakeSpeed) {
    // Set the "intake" of this command equal to the intake argument that was passed in. This makes
    // the variable global (inside of this command).
    this.intake = intake;

    // Set the speed of this command equal to the speed that was passed in, making it global (in
    // this command).
    this.intakeSpeed = intakeSpeed;

    // Add the intake subsystem as a requirement of this command, prevents from running any other
    // commands that require the intake at the same time as this one.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Call the intake subsystem and set the speed
    intake.setSpeed(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Once this command ends, set the intake speed to zero.
    intake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
