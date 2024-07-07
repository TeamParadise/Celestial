// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/** Main robot class. */
@SuppressWarnings("DataFlowIssue")
public class Robot extends LoggedRobot {
  // Create autonomous command variable to store selected autonomous
  private Command autonomousCommand;

  // Create RobotContainer object to create subsystems and controller bindings
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    // Record build metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("RobotMode", BuildConstants.MAVEN_GROUP);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers, and if replaying, replay source
    switch (Constants.robotMode) {
      case Sim:
        // Running in a simulation, only log to NetworkTables
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case Replay:
        // Replaying a log, set up the source of the replay
        setUseTiming(false); // Run the log as fast as possible
        // Setup replay source and new log location
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;

      default:
        // Assume real robot if nothing else is true, log to USB and NetworkTables
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
    }

    // Start the logger
    Logger.start();

    // Initialize the RobotContainer
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  // The following functions are to check the robot mode using the robotMode in Constants
  /**
   * Get if the robot is real, according to the Constants.robotMode.
   *
   * <p>Returns: If the code is for a real robot.
   */
  public static boolean isRealAK() {
    return Constants.robotMode == Mode.Real;
  }

  /**
   * Get if the robot is a simulation, according to the Constants.robotMode.
   *
   * <p>Returns: If the code is for a simulation.
   */
  public static boolean isSimulationAK() {
    return Constants.robotMode == Mode.Sim;
  }

  /**
   * Get if the robot is a replay, according to the Constants.robotMode.
   *
   * <p>Returns: If the code is for a replay.
   */
  public static boolean isReplayAK() {
    return Constants.robotMode == Mode.Replay;
  }
}
