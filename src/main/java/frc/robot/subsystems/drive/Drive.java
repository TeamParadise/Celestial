// File originally made by: Mechanical Advantage - FRC 6328
// Copyright (c) 2024 FRC 6328 (https://github.com/Mechanical-Advantage)
// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.io.DriveIO;
import frc.robot.subsystems.drive.io.DriveIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Class for controlling a Differential Drivetrain. */
public class Drive extends SubsystemBase {
  // DriveIO objects
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  // GyroIO objects
  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  // Create odometry and kinematics objeects
  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(DriveConstants.trackWidth);

  // Create a SysID object
  private SysIdRoutine sysId;

  /** Class for controlling a Differential Drivetrain. */
  public Drive(DriveIO io, GyroIO gyroIO) {
    // Set this classes IO objects equal to the provided IO classes
    this.io = io;
    this.gyroIO = gyroIO;

    // Call function to configure PathPlanner and SysID
    configure();
  }

  /** Class for controlling a Differential Drivetrain in a simulation. */
  public Drive(DriveIO io) {
    // Set this classes IO objects equal to the provided IO classes
    this.io = io;

    // Call function to configure PathPlanner and SysID
    configure();
  }

  private void configure() {
    // Configure AutoBuilder using Ramsete for PathPlanner
    AutoBuilder.configureRamsete(
        this::getPose, // Method to get current pose
        this::setPose, // Method to set pose
        () ->
            kinematics.toChassisSpeeds(
                new DifferentialDriveWheelSpeeds(
                    getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec())),
        (speeds) -> {
          var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
          driveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
        },
        new ReplanningConfig(),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);

    // Create a SysID routine
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> driveVolts(voltage.in(Volts), voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // Update and log inputs from the drivetrain
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    // Use AdvantageKit modes for this eventually
    if (Robot.isReal()) {
      // Update and log inputs from the gyro and update odometry
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      odometry.update(gyroInputs.yaw, getLeftPositionMeters(), getRightPositionMeters());
    } else {
      // No gyro is used in simulation, so use the simulated yaw
      odometry.update(inputs.simulatedYaw, getLeftPositionMeters(), getRightPositionMeters());
    }
  }

  /** Tell the drivetrain to move in a open-loop manner at a certain number of volts. */
  public void driveVolts(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  /** Tell the drivetrain to move in an open-loop manner with arcade-style controls. */
  public void driveArcade(double xSpeed, double zRotation) {
    // Use kinematics to figure out the speeds of the left and right side of the chassis
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
    io.setSpeed(speeds.left, speeds.right);
  }

  /** Tell the drivetrain to move in a closed-loop manner with velocity control. */
  public void driveVelocity(double leftMetersPerSec, double rightMetersPerSec) {
    Logger.recordOutput("Drive/ClosedLoop/Active", true);
    Logger.recordOutput("Drive/LeftVelocitySetpointMetersPerSec", leftMetersPerSec);
    Logger.recordOutput("Drive/RightVelocitySetpointMetersPerSec", rightMetersPerSec);

    io.setVelocity(leftMetersPerSec, rightMetersPerSec);
  }

  /** Returns the current odometry pose in meters. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    if (Robot.isReal()) {
      odometry.resetPosition(
          gyroInputs.yaw, getLeftPositionMeters(), getRightPositionMeters(), pose);
    } else {
      odometry.resetPosition(
          inputs.simulatedYaw, getLeftPositionMeters(), getRightPositionMeters(), pose);
    }
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
