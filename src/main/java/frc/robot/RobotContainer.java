// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.io.DriveIO;
import frc.robot.subsystems.drive.io.DriveIOSim;
import frc.robot.subsystems.drive.io.DriveIOSparkMax;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.io.FlywheelsIO;
import frc.robot.subsystems.flywheels.io.FlywheelsIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOSparkMax;

/** Main robot class for setting controller bindings, and running commands. */
public class RobotContainer {
  // Create subsystems
  private final Drive drive;
  private final Intake intake;
  private final Flywheels flywheels;

  // Create controller(s)
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController coDriverController = new CommandXboxController(1);

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
        drive = new Drive(new DriveIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        flywheels = new Flywheels(new FlywheelsIOSparkMax());
        break;
    }

    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {}

  private void configureDefaultCommands() {
    drive.setDefaultCommand(
        DriveCommands.driveArcade(
            drive, () -> -driverController.getLeftY(), driverController::getRightX));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
