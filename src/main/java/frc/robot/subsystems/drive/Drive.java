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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveConstants.RealConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.io.DriveIO;
import frc.robot.subsystems.drive.io.DriveIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Class for controlling a Differential Drivetrain. */
public class Drive extends SubsystemBase {
  // DriveIO objects
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  // GyroIO objects
  private GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  // Create odometry and kinematics objects
  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(DriveConstants.trackWidth);

  // Create a SysID object
  private SysIdRoutine sysId;

  // Set PID values for the left drive to allow them to be tuned
  private static final LoggedTunableNumber leftP =
      new LoggedTunableNumber("Drive/Tuning/Left/P", RealConstants.leftP);
  private static final LoggedTunableNumber leftI =
      new LoggedTunableNumber("Drive/Tuning/Left/I", RealConstants.leftI);
  private static final LoggedTunableNumber leftD =
      new LoggedTunableNumber("Drive/Tuning/Left/D", RealConstants.leftD);
  private static final LoggedTunableNumber leftF =
      new LoggedTunableNumber("Drive/Tuning/Left/F", RealConstants.leftF);

  // Set PID values for the right drive to allow them to be tuned
  private static final LoggedTunableNumber rightP =
      new LoggedTunableNumber("Drive/Tuning/Right/P", RealConstants.rightP);
  private static final LoggedTunableNumber rightI =
      new LoggedTunableNumber("Drive/Tuning/Right/I", RealConstants.rightI);
  private static final LoggedTunableNumber rightD =
      new LoggedTunableNumber("Drive/Tuning/Right/D", RealConstants.rightD);
  private static final LoggedTunableNumber rightF =
      new LoggedTunableNumber("Drive/Tuning/Right/F", RealConstants.rightF);

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

    // Update PIDF values if changed in tuning mode
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setLeftPIDF(leftP.get(), leftI.get(), leftD.get(), leftF.get()),
        leftP,
        leftI,
        leftD,
        leftF);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setRightPIDF(rightP.get(), rightI.get(), rightD.get(), rightF.get()),
        rightP,
        rightI,
        rightD,
        rightF);

    // Check to make sure if the robot is real
    if (Robot.isRealAK()) {
      // Update and log inputs from the gyro and update odometry
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      odometry.update(gyroInputs.yaw, getLeftPositionMeters(), getRightPositionMeters());
    } else {
      // No gyro is used in simulation, so use the simulated yaw
      odometry.update(inputs.simulatedYaw, getLeftPositionMeters(), getRightPositionMeters());
    }
  }

  // Basic drive functions to be used in commands
  /**
   * Tell the drivetrain to move in an open-loop manner at a certain number of volts.
   * <p><font color="red">
   * Shouldn't be used directly without going through a Command.
   * </font>
   */
  public void driveVolts(double leftVolts, double rightVolts) {
    // Log that we are no longer running in closed loop mode
    Logger.recordOutput("Drive/ClosedLoop/Active", false);

    io.setVoltage(leftVolts, rightVolts);
  }

  /**
   * Tell the drivetrain to move in an open-loop manner with arcade-style controls.
   * <p><font color="red">
   * Shouldn't be used directly without going through a Command.
   * </font>
   */
  public void driveArcade(double xSpeed, double zRotation) {
    // Log that we are no longer running in closed loop mode
    Logger.recordOutput("Drive/ClosedLoop/Active", false);

    // Use kinematics to figure out the speeds of the left and right side of the chassis
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
    io.setSpeed(speeds.left, speeds.right);
  }

  /**
   * Tell the drivetrain to move in a closed-loop manner with velocity control.
   * <p><font color="red">
   * Shouldn't be used directly without going through a Command.
   * </font>
   */
  public void driveVelocity(double leftMetersPerSec, double rightMetersPerSec) {
    // Log that we are running in closed-loop mode and log setpoints
    Logger.recordOutput("Drive/ClosedLoop/Active", true);
    Logger.recordOutput("Drive/LeftVelocitySetpointMetersPerSec", leftMetersPerSec);
    Logger.recordOutput("Drive/RightVelocitySetpointMetersPerSec", rightMetersPerSec);

    io.setVelocity(leftMetersPerSec, rightMetersPerSec);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current odometry pose in meters. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    // If the robot is real, use gyro inputs, otherwise, use simulated
    if (Robot.isRealAK()) {
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
    return inputs.leftVelocityRPM * DriveConstants.metersPerRotation;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  @AutoLogOutput
  public double getRightVelocityMetersPerSec() {
    return inputs.rightVelocityRPM * DriveConstants.metersPerRotation;
  }
}
