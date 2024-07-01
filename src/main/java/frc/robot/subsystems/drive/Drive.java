// File originally made by: Mechanical Advantage - FRC 6328
// Copyright (c) 2024 FRC 6328 (https://github.com/Mechanical-Advantage)
// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.io.DriveIO;
import frc.robot.subsystems.drive.io.DriveIOInputsAutoLogged;

/** Class for controlling a Differential Drivetrain. */
public class Drive extends SubsystemBase {
  // DriveIO objects
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  // Create odometry and kinematics objeects
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);

  /** Class for controlling a Differential Drivetrain. */
  public Drive(DriveIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update and log inputs from the drivetrain
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    // Update the odometry of the drivetrain
    odometry.update(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters());
  }

  /** Tell the drivetrain to move in an open-loop manner with arcade-style controls.*/
  public void driveArcade(double xSpeed, double zRotation) {
    // Use kinematics to figure out the speeds of the left and right side of the chassis
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
    io.setSpeed(speeds.left, speeds.right);
  }

  /** Returns the current odometry pose in meters. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    odometry.resetPosition(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters(), pose);
  }

  /** Returns the position of the left wheels in meters. */
  @AutoLogOutput
  public double getLeftPositionMeters() {
    return inputs.leftPositionRotations * DriveConstants.metersPerRotation;
  }

  /** Returns the position of the right wheels in meters. */
  @AutoLogOutput
  public double getRightPositionMeters() {
    return inputs.rightPositionRotations * DriveConstants.metersPerRotation;
  }

  /** Returns the velocity of the left wheels in meters/second. */
  @AutoLogOutput
  public double getLeftVelocityMetersPerSec() {
    return inputs.leftVelocityRotationsPerSec * DriveConstants.metersPerRotation;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  @AutoLogOutput
  public double getRightVelocityMetersPerSec() {
    return inputs.rightVelocityRotationsPerSec * DriveConstants.metersPerRotation;
  }
}
