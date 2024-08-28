// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

/** Constants to be used throughout the whole robot. */
@SuppressWarnings("ConstantValue")
public class Constants {
  // If simulated, set to simulation, if replay, set to replay, otherwise, set to real
  public static final Mode robotMode =
      BuildConstants.MAVEN_GROUP.equals("simulation")
          ? Mode.Sim
          : BuildConstants.MAVEN_GROUP.equals("replay") ? Mode.Replay : Mode.Real;
  public static final boolean tuningMode = false;
  public static final double loopPeriodSecs = 0.02;

  /** Constants to be used for the Xbox controllers for the robot */
  public static class XboxConstants {
    public static final double driverControllerID = 0;
    public static final double operatorControllerID = 1;

    public static final double joystickDeadband = 0.1;
  }

  public enum Mode {
    /** Running on a real robot. */
    Real,

    /** Running in a simulation. */
    Sim,

    /** Replaying from a log file. */
    Replay
  }
}
