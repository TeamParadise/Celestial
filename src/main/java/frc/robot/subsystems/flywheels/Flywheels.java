// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.flywheels.FlywheelsConstants.*;
import frc.robot.subsystems.flywheels.io.FlywheelsIO;
import frc.robot.subsystems.flywheels.io.FlywheelsIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Class for controlling a set of Flywheels. */
public class Flywheels extends SubsystemBase {
  // FlywheelsIO objects
  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  // Set PID values for the bottom intake motor to allow them to be tuned
  private static final LoggedTunableNumber bottomP =
      new LoggedTunableNumber("Flywheels/Bottom/P", BottomConstants.bottomP);
  private static final LoggedTunableNumber bottomI =
      new LoggedTunableNumber("Flywheels/Bottom/I", BottomConstants.bottomI);
  private static final LoggedTunableNumber bottomD =
      new LoggedTunableNumber("Flywheels/Bottom/D", BottomConstants.bottomD);
  private static final LoggedTunableNumber bottomF =
      new LoggedTunableNumber("Flywheels/Bottom/F", BottomConstants.bottomF);
  private static final LoggedTunableNumber bottomIz =
      new LoggedTunableNumber("Flywheels/Bottom/Iz", BottomConstants.bottomIz);

  // Set PID values for the bottom intake motor to allow them to be tuned
  private static final LoggedTunableNumber topP =
      new LoggedTunableNumber("Flywheels/Top/P", TopConstants.topP);
  private static final LoggedTunableNumber topI =
      new LoggedTunableNumber("Flywheels/Top/I", TopConstants.topI);
  private static final LoggedTunableNumber topD =
      new LoggedTunableNumber("Flywheels/Top/D", TopConstants.topD);
  private static final LoggedTunableNumber topF =
      new LoggedTunableNumber("Flywheels/Top/F", TopConstants.topF);
  private static final LoggedTunableNumber topIz =
      new LoggedTunableNumber("Flywheels/Top/Iz", TopConstants.topIz);

  /** Class for controlling a set of Flywheels. */
  public Flywheels(FlywheelsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update and log inputs from the intake
    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);

    // Update PIDF values if changed in tuning mode
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setBottomPIDF(bottomP.get(), bottomI.get(), bottomD.get(), bottomF.get(), bottomIz.get()),
        bottomP,
        bottomI,
        bottomD,
        bottomF,
        bottomIz);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setTopPIDF(topP.get(), topI.get(), topD.get(), topF.get(), topIz.get()),
        topP,
        topI,
        topD,
        topF,
        topIz);
  }

  // Basic flywheel methods to be used in commands
  /**
   * Tell the flywheels to run in an open-loop manner at a certain number of volts.
   *
   * <p><font color="red"> Shouldn't be used directly without going through a Command. </font>
   */
  public void setVolts(double flywheelVolts) {
    // Log that we are no longer running in open-loop mode
    Logger.recordOutput("Flywheels/ClosedLoop/Active", false);

    io.setVoltage(flywheelVolts);
  }

  /**
   * Tell the flywheels to run in an open-loop manner at a certain speed.
   *
   * <p><font color="red"> Shouldn't be used directly without going through a Command. </font>
   */
  public void setSpeed(double flywheelSpeed) {
    // Log that we are no longer running in open-loop mode
    Logger.recordOutput("Intake/ClosedLoop/Active", false);

    io.setSpeed(flywheelSpeed);
  }

  /**
   * Tell the flywheels to run in a closed-loop manner with velocity control.
   *
   * <p><font color="red"> Shouldn't be used directly without going through a Command. </font>
   */
  public void setVelocity(double flywheelRPM) {
    // Log that we are running in closed-loop mode and log setpoints
    Logger.recordOutput("Flywheels/ClosedLoop/Active", true);
    Logger.recordOutput("Flywheels/VelocitySetpointRPM", flywheelRPM);

    io.setVelocity(flywheelRPM);
  }

  /** Returns the average velocity of the flywheels in rotations per minute (RPM). */
  @AutoLogOutput
  public double getFlywheelVelocity() {
    return (inputs.bottomVelocityRPM + inputs.topVelocityRPM) / 2;
  }
}
