// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualCommand;
import frc.robot.commands.PassNote;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootNote;
import frc.robot.commands.auto.TimedTwoNote;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIONavX;
import frc.robot.subsystems.drive.io.DriveIO;
import frc.robot.subsystems.drive.io.DriveIOSim;
import frc.robot.subsystems.drive.io.DriveIOSparkMax;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsConstants;
import frc.robot.subsystems.flywheels.io.FlywheelsIO;
import frc.robot.subsystems.flywheels.io.FlywheelsIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Main robot class for setting controller bindings, and running commands. */
public class RobotContainer {
  // Create subsystems
  private final Drive drive;
  private final Intake intake;
  private final Flywheels flywheels;

  // Create controller(s)
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController coDriverController = new CommandXboxController(1);

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    // Set up all the subsystems depending on real, simulation, or replay
    switch (Constants.robotMode) {
      case Sim:
        // Simulated robot, set up the simulated hardware classes
        // Note, Intake and Flywheels do not have simulated classes yet, we are just using blank
        // IO classes. Also, the drive kitbot classes will likely need to be tuned.
        drive =
            new Drive(
                new DriveIOSim(
                    KitbotMotor.kDoubleNEOPerSide, KitbotGearing.k8p45, KitbotWheelSize.kSixInch));
        intake = new Intake(new IntakeIO() {});
        flywheels = new Flywheels(new FlywheelsIO() {});
        break;

      case Replay:
        // Replayed robot, set up the replay hardware classes (empty IO)
        drive = new Drive(new DriveIO() {});
        intake = new Intake(new IntakeIO() {});
        flywheels = new Flywheels(new FlywheelsIO() {});
        break;

      default:
        // Assume real robot if nothing else is true, set up real hardware classes
        drive = new Drive(new DriveIOSparkMax(), new GyroIONavX());
        intake = new Intake(new IntakeIOSparkMax());
        flywheels = new Flywheels(new FlywheelsIOSparkMax());
        break;
    }

    configureAutoCommands();
    configureDriverController();
    configureCoDriverController();
    configureDefaultCommands();

    // Configure basic auto chooser with PathPlanner autos
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    configureAutoChooser();
  }

  private void configureAutoCommands() {
    // Intake commands
    NamedCommands.registerCommand("Intake", new IntakeCommand(flywheels, intake).withTimeout(5));
    NamedCommands.registerCommand("Intake (No Timeout)", new IntakeCommand(flywheels, intake));

    // Shoot/toss commands
    NamedCommands.registerCommand(
        "Shoot Amp",
        new ShootCommand(
                flywheels, intake, FlywheelsConstants.Presets.amp, IntakeConstants.Presets.feed)
            .withTimeout(5));
    NamedCommands.registerCommand(
        "Shoot Speaker", new ShootCommand(flywheels, intake).withTimeout(5));
    NamedCommands.registerCommand(
        "Toss",
        new ShootCommand(
                flywheels, intake, FlywheelsConstants.Presets.toss, IntakeConstants.Presets.feed)
            .withTimeout(5));
  }

  private void configureAutoChooser() {
    // Add SysID routines to the auto chooser, if we are in tuning mode
    if (Constants.tuningMode) {
      autoChooser.addOption(
          "Flywheels SysId 1 Forward", flywheels.sysIdQuasistatic(Direction.kForward));
      autoChooser.addOption(
          "Flywheels SysId 1 Backwards", flywheels.sysIdQuasistatic(Direction.kReverse));
      autoChooser.addOption(
          "Flywheels SysId 2 Forward", flywheels.sysIdDynamic(Direction.kForward));
      autoChooser.addOption(
          "Flywheels SysId 2 Backwards", flywheels.sysIdDynamic(Direction.kReverse));

      autoChooser.addOption(
          "Drivetrain SysId 1 Forward", drive.sysIdQuasistatic(Direction.kForward));
      autoChooser.addOption(
          "Drivetrain SysId 1 Backwards", drive.sysIdQuasistatic(Direction.kReverse));
      autoChooser.addOption("Drivetrain SysId 2 Forward", drive.sysIdDynamic(Direction.kForward));
      autoChooser.addOption("Drivetrain SysId 2 Backwards", drive.sysIdDynamic(Direction.kReverse));
    }

    autoChooser.addOption(
        "THIS THE AUTO USE THIS ONE!!!!!!", new TimedTwoNote(drive, intake, flywheels));
  }

  private void configureDriverController() {
    // Bindings for the ABXY buttons
    // A Button - Shoot into Speaker
    driverController.a().onTrue(new ShootCommand(flywheels, intake));
    // X Button - Manual Shoot (for getting any really stuck notes out)
    driverController
        .x()
        .onTrue(
            new ManualCommand(
                flywheels, intake, FlywheelsConstants.Presets.toss, IntakeConstants.Presets.feed));
    // Y Button - Passing Shoot
    driverController.b().onTrue(new ShootNote(intake, flywheels));
    driverController.y().onTrue(new PassNote(intake, flywheels));
  }

  private void configureCoDriverController() {
    // Bindings for the ABXY buttons
    // A Button - Shoot into Amp
    coDriverController.a().onTrue(new AmpCommand(flywheels, intake));
    // B Button - Auto Intake (ends when a note is detected in the intake)
    coDriverController.b().whileTrue(new IntakeCommand(flywheels, intake));

    // Bindings for triggers
    // Left Trigger - Reverse Intake and Flywheels (shoot out of the back of robot)
    coDriverController
        .leftTrigger(0.5)
        .whileTrue(
            new ManualCommand(
                flywheels,
                intake,
                FlywheelsConstants.Presets.retract,
                IntakeConstants.Presets.retract));
    // Right Trigger - Manual Intake (only ends when the button is let go)
    coDriverController.rightTrigger(0.5).whileTrue(new ManualCommand(flywheels, intake));
  }

  private void configureDefaultCommands() {
    // Set the drivetrain to be controlled via the driver controller joysticks
    drive.setDefaultCommand(
        DriveCommands.driveArcade(
            drive, () -> -driverController.getLeftY(), () -> -driverController.getRightX()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
