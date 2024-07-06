// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import frc.robot.Constants.Mode;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Field2d field2d = new Field2d();

  @Override
  public void robotInit() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
    m_robotContainer = new RobotContainer();

    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    field2d.setRobotPose(m_robotContainer.drive.getPose());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
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
  /** Get if the robot is real, according to the Constants.robotMode.
   * <p>
   * Returns: If the code is for a real robot.
   */
  public static boolean isRealAK() {
    return Constants.robotMode == Mode.Real;
  }

  /** Get if the robot is a simulation, according to the Constants.robotMode.
   * <p>
   * Returns: If the code is for a simulation.
   */
  public static boolean isSimulationAK() {
    return Constants.robotMode == Mode.Sim;
  }

  /** Get if the robot is a replay, according to the Constants.robotMode.
   * <p>
   * Returns: If the code is for a replay.
   */
  public static boolean isReplayAK() {
    return Constants.robotMode == Mode.Replay;
  }
}
