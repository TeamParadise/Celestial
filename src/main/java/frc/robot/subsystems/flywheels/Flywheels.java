// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  // Create a SysID object to run SysID routines
  private SysIdRoutine sysId;

  // Set PIDF values for the bottom flywheel motor to allow them to be tuned
  private static final LoggedTunableNumber bottomP =
      new LoggedTunableNumber("Flywheels/Bottom/P", BottomConstants.bottomP);
  private static final LoggedTunableNumber bottomI =
      new LoggedTunableNumber("Flywheels/Bottom/I", BottomConstants.bottomI);
  private static final LoggedTunableNumber bottomD =
      new LoggedTunableNumber("Flywheels/Bottom/D", BottomConstants.bottomD);
  private static final LoggedTunableNumber bottomS =
      new LoggedTunableNumber("Flywheels/Bottom/S", BottomConstants.bottomS);
  private static final LoggedTunableNumber bottomV =
      new LoggedTunableNumber("Flywheels/Bottom/V", BottomConstants.bottomV);
  private static final LoggedTunableNumber bottomA =
      new LoggedTunableNumber("Flywheels/Bottom/A", BottomConstants.bottomA);
  private static final LoggedTunableNumber bottomIz =
      new LoggedTunableNumber("Flywheels/Bottom/Iz", BottomConstants.bottomIz);
  private static final LoggedTunableNumber bottomMaxAccel =
      new LoggedTunableNumber("Flywheels/Bottom/MaxAccel", BottomConstants.bottomMaxAccel);

  // Set PIDF values for the top flywheel motor to allow them to be tuned
  private static final LoggedTunableNumber topP =
      new LoggedTunableNumber("Flywheels/Top/P", TopConstants.topP);
  private static final LoggedTunableNumber topI =
      new LoggedTunableNumber("Flywheels/Top/I", TopConstants.topI);
  private static final LoggedTunableNumber topD =
      new LoggedTunableNumber("Flywheels/Top/D", TopConstants.topD);
  private static final LoggedTunableNumber topS =
      new LoggedTunableNumber("Flywheels/Top/S", TopConstants.topS);
  private static final LoggedTunableNumber topV =
      new LoggedTunableNumber("Flywheels/Top/V", TopConstants.topV);
  private static final LoggedTunableNumber topA =
      new LoggedTunableNumber("Flywheels/Top/A", TopConstants.topA);
  private static final LoggedTunableNumber topIz =
      new LoggedTunableNumber("Flywheels/Top/Iz", TopConstants.topIz);
  private static final LoggedTunableNumber topMaxAccel =
      new LoggedTunableNumber("Flywheels/Top/MaxAccel", TopConstants.topMaxAccel);

  /** Class for controlling a set of Flywheels. */
  public Flywheels(FlywheelsIO io) {
    // Set this classes IO objects equal to the provided IO classes
    this.io = io;

    // Create a SysID routine
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(2).per(Seconds.of(1)),
                null,
                null,
                (state) -> Logger.recordOutput("Flywheels/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // Update and log inputs from the intake
    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);

    // Update PIDF values if changed in tuning mode
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            io.setBottomPIDF(
                bottomP.get(),
                bottomI.get(),
                bottomD.get(),
                bottomS.get(),
                bottomV.get(),
                bottomA.get(),
                bottomIz.get(),
                bottomMaxAccel.get()),
        bottomP,
        bottomI,
        bottomD,
        bottomS,
        bottomV,
        bottomA,
        bottomIz,
        bottomMaxAccel);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            io.setTopPIDF(
                topP.get(),
                topI.get(),
                topD.get(),
                topS.get(),
                topV.get(),
                topA.get(),
                topIz.get(),
                topMaxAccel.get()),
        topP,
        topI,
        topD,
        topS,
        topV,
        topA,
        topIz,
        topMaxAccel);
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
    Logger.recordOutput("Flywheels/DifferentSetpoints", false);
    Logger.recordOutput("Flywheels/VelocitySetpointRPM", flywheelRPM);

    io.setVelocity(flywheelRPM);
  }

  /**
   * Tell the flywheels to run in a closed-loop manner with velocity control.
   *
   * <p><font color="red"> Shouldn't be used directly without going through a Command. </font>
   */
  public void setVelocity(double bottomRPM, double topRPM) {
    // Log that we are running in closed-loop mode and log setpoints
    Logger.recordOutput("Flywheels/ClosedLoop/Active", true);
    Logger.recordOutput("Flywheels/DifferentSetpoints", true);
    Logger.recordOutput("Flywheels/VelocitySetpointRPM", bottomRPM);
    Logger.recordOutput("Flywheels/TopVelocitySetpointRPM", topRPM);

    io.setVelocity(bottomRPM, topRPM);
  }

  /** Returns a command to run a quasistatic SysID test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic SysID test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the average velocity of the flywheels in rotations per minute (RPM). */
  @AutoLogOutput
  public double getFlywheelVelocity() {
    return (inputs.bottomVelocityRPM + inputs.topVelocityRPM) / 2;
  }

  /** Returns the velocity of the top flywheel in rotations per minute (RPM). */
  public double getTopFlywheelVelocity() {
    return inputs.topVelocityRPM;
  }

  /** Returns the velocity of the bottom flywheel in rotations per minute (RPM). */
  public double getBottomFlywheelVelocity() {
    return inputs.bottomVelocityRPM;
  }
}
