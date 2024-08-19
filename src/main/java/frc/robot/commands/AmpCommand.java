// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.LoggedTunableNumber;

public class AmpCommand extends Command {
  // Subsystems used in this command
  private final Flywheels flywheels;
  private final Intake intake;

  // Logged and tunable values
  private final LoggedTunableNumber intakeRPM;
  private final LoggedTunableNumber flywheelRPM;
  private final LoggedTunableNumber flywheelSpeedupTime =
      new LoggedTunableNumber("Commands/AmpCommand/FlywheelSpeedupTime", 400000);
  private final LoggedTunableNumber intakeFeedTime =
      new LoggedTunableNumber("Commands/AmpCommand/IntakeFeedTime", 4500000);
  private final LoggedTunableNumber topFlywheelSlowdown =
      new LoggedTunableNumber("Commands/AmpCommand/TopFlywheelSlowDown", 1200);

  // Create items to track the amount of time the command has been running for
  private double timeElapsed = 0.0;
  private double previousTimestamp = 0.0;

  // Used to reduce false positives or negatives by making sure something is true for long enough.
  // This is used to detect if our flywheels are up to speed
  private final Debouncer bottomDebouncer = new Debouncer(0.15, DebounceType.kBoth);
  private final Debouncer topDebouncer = new Debouncer(0.15, DebounceType.kBoth);

  public AmpCommand(Flywheels flywheels, Intake intake, double flywheelRPM, double intakeRPM) {
    this.flywheels = flywheels;
    this.intake = intake;

    this.flywheelRPM = new LoggedTunableNumber("Commands/AmpCommand/FlywheelRPM", flywheelRPM);
    this.intakeRPM = new LoggedTunableNumber("Commands/AmpCommand/IntakeRPM", intakeRPM);

    addRequirements(this.flywheels, this.intake);
  }

  public AmpCommand(Flywheels flywheels, Intake intake) {
    this.flywheels = flywheels;
    this.intake = intake;

    // If no speed is provided, assume we are shooting into the speaker
    this.flywheelRPM =
        new LoggedTunableNumber(
            "Commands/ShootCommand/FlywheelRPM", FlywheelsConstants.Presets.amp);
    this.intakeRPM =
        new LoggedTunableNumber("Commands/ShootCommand/IntakeRPM", IntakeConstants.Presets.feed);

    addRequirements(this.flywheels, this.intake);
  }

  @Override
  public void initialize() {
    // Set to intake to run backwards for a short while to make sure that the note is not touching
    // the flywheels when we start speeding them up
    intake.setVelocity(IntakeConstants.Presets.retract);

    // Reset time elapsed values
    timeElapsed = 0.0;
    previousTimestamp = RobotController.getFPGATime();
  }

  @Override
  public void execute() {
    // Set flywheel RPM and flywheel velocity for this loop
    double bottomFlywheelSetpoint = flywheelRPM.get();
    double topFlywheelSetpoint = bottomFlywheelSetpoint - topFlywheelSlowdown.get();
    double bottomFlywheelVelocity = flywheels.getBottomFlywheelVelocity();
    double topFlywheelVelocity = flywheels.getTopFlywheelVelocity();

    // Set velocity of flywheels and intake depending on time elapsed here
    if (timeElapsed < 250000) {
      intake.setVelocity(IntakeConstants.Presets.retract);
    } else if (timeElapsed < 4000000
        && !bottomDebouncer.calculate(
            bottomFlywheelVelocity > bottomFlywheelSetpoint - Math.pow(bottomFlywheelSetpoint, 0.55)
                && bottomFlywheelVelocity
                    < bottomFlywheelSetpoint + Math.pow(bottomFlywheelSetpoint, 0.55))
        && !topDebouncer.calculate(
            topFlywheelVelocity > topFlywheelSetpoint - Math.pow(topFlywheelSetpoint, 0.55)
                && topFlywheelVelocity
                    < topFlywheelSetpoint + Math.pow(topFlywheelSetpoint, 0.55))) {
      flywheels.setVelocity(bottomFlywheelSetpoint, topFlywheelSetpoint);
      intake.setVelocity(0);
    } else {
      // Just set time elapsed to 4 seconds if it is not 4 seconds already, so there is 1 second
      // left to shoot the note
      if (timeElapsed < 4000000) {
        timeElapsed = 4000000;
      }

      flywheels.setVelocity(bottomFlywheelSetpoint, topFlywheelSetpoint);
      intake.setVelocity(intakeRPM.get());
    }

    // Calculate the time that has elapsed since the command has started
    timeElapsed += RobotController.getFPGATime() - previousTimestamp;
    previousTimestamp = RobotController.getFPGATime();
  }

  @Override
  public boolean isFinished() {
    return timeElapsed >= 5000000;
  }

  @Override
  public void end(boolean interrupted) {
    flywheels.setSpeed(0);
    intake.setSpeed(0);
  }
}
