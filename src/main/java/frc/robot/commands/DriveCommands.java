// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

/**
 * A class used to send commands to a {@link Drive} subsystem. Will be replaced with DriveRequest in
 * the future.
 */
public class DriveCommands {
  /**
   * Tells the drivetrain to move in an open-loop manner at a certain number of volts.
   *
   * @param drive The {@link Drive} subsystem to use for this command.
   * @param leftVoltsSupplier A {@link DoubleSupplier} that provides the left voltage to apply.
   * @param rightVoltsSupplier A {@link DoubleSupplier} that provides the right voltage to apply.
   */
  public static Command driveVolts(
      Drive drive, DoubleSupplier leftVoltsSupplier, DoubleSupplier rightVoltsSupplier) {
    return Commands.run(
        () -> drive.driveVolts(leftVoltsSupplier.getAsDouble(), rightVoltsSupplier.getAsDouble()),
        drive);
  }

  /**
   * Tells the drivetrain to move in an open-loop manner with arcade-style controls.
   *
   * @param drive The {@link Drive} subsystem to use for this command.
   * @param speedSupplier A {@link DoubleSupplier} that provides the speed to apply.
   * @param rotationSupplier A {@link DoubleSupplier} that provides the right rotation to apply.
   */
  public static Command driveArcade(
      Drive drive, DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier) {
    return Commands.run(
        () -> drive.driveArcade(speedSupplier.getAsDouble(), rotationSupplier.getAsDouble()),
        drive);
  }

  /**
   * Tells the drivetrain to move in a closed-loop manner with velocity control.
   *
   * @param drive The {@link Drive} subsystem to use for this command.
   * @param leftMetersPerSecSupplier A {@link DoubleSupplier} that provides the left velocity to
   *     apply, in meters per second.
   * @param rightMetersPerSecSupplier A {@link DoubleSupplier} that provides the right velocity to
   *     apply, in meters per second.
   */
  public static Command driveVelocity(
      Drive drive,
      DoubleSupplier leftMetersPerSecSupplier,
      DoubleSupplier rightMetersPerSecSupplier) {
    return Commands.run(
        () ->
            drive.driveVelocity(
                leftMetersPerSecSupplier.getAsDouble(), rightMetersPerSecSupplier.getAsDouble()),
        drive);
  }
}
