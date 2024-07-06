// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.io.DriveIOSim;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.io.FlywheelsIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.io.IntakeIOSparkMax;

public class RobotContainer {
  // Create subsystems
  public static Drive drive;
  public static Intake intake;
  public static Flywheels flywheels;

  // Create controller(s)
  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    drive =
        new Drive(
            new DriveIOSim(
                KitbotMotor.kDoubleNEOPerSide, KitbotGearing.k8p45, KitbotWheelSize.kSixInch));
    intake = new Intake(new IntakeIOSparkMax());
    flywheels = new Flywheels(new FlywheelsIOSparkMax());
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        Commands.run(
            () -> drive.driveArcade(-controller.getLeftY(), controller.getRightX()), drive));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
