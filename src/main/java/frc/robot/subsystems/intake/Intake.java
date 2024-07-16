// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.*;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Class for controlling a basic dual-motor Intake. */
public class Intake extends SubsystemBase {
  // IntakeIO objects
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // Set PID values for the bottom intake motor to allow them to be tuned
  private static final LoggedTunableNumber bottomP =
      new LoggedTunableNumber("Intake/Bottom/P", BottomConstants.bottomP);
  private static final LoggedTunableNumber bottomI =
      new LoggedTunableNumber("Intake/Bottom/I", BottomConstants.bottomI);
  private static final LoggedTunableNumber bottomD =
      new LoggedTunableNumber("Intake/Bottom/D", BottomConstants.bottomD);
  private static final LoggedTunableNumber bottomF =
      new LoggedTunableNumber("Intake/Bottom/F", BottomConstants.bottomF);

  // Set PID values for the bottom intake motor to allow them to be tuned
  private static final LoggedTunableNumber topP =
      new LoggedTunableNumber("Intake/Top/P", TopConstants.topP);
  private static final LoggedTunableNumber topI =
      new LoggedTunableNumber("Intake/Top/I", TopConstants.topI);
  private static final LoggedTunableNumber topD =
      new LoggedTunableNumber("Intake/Top/D", TopConstants.topD);
  private static final LoggedTunableNumber topF =
      new LoggedTunableNumber("Intake/Top/F", TopConstants.topF);

  /** Class for controlling a basic dual-motor Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update and log inputs from the intake
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Update PIDF values if changed in tuning mode
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setBottomPIDF(bottomP.get(), bottomI.get(), bottomD.get(), bottomF.get()),
        bottomP,
        bottomI,
        bottomD,
        bottomF);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setTopPIDF(topP.get(), topI.get(), topD.get(), topF.get()),
        topP,
        topI,
        topD,
        topF);
  }

  // Basic intake methods to be used in commands
  /**
   * Tell the intake to run in an open-loop manner at a certain number of volts.
   *
   * <p><font color="red"> Shouldn't be used directly without going through a Command. </font>
   */
  public void setVolts(double intakeVolts) {
    // Log that we are no longer running in open-loop mode
    Logger.recordOutput("Intake/ClosedLoop/Active", false);

    io.setVoltage(intakeVolts);
  }

  /**
   * Tell the intake to run in an open-loop manner at a certain speed.
   *
   * <p><font color="red"> Shouldn't be used directly without going through a Command. </font>
   */
  public void setSpeed(double intakeSpeed) {
    // Log that we are no longer running in open-loop mode
    Logger.recordOutput("Intake/ClosedLoop/Active", false);

    io.setSpeed(intakeSpeed);
  }

  /**
   * Tell the intake to run in a closed-loop manner with velocity control.
   *
   * <p><font color="red"> Shouldn't be used directly without going through a Command. </font>
   */
  public void setVelocity(double intakeRPM) {
    // Log that we are running in closed-loop mode and log setpoints
    Logger.recordOutput("Intake/ClosedLoop/Active", true);
    Logger.recordOutput("Intake/VelocitySetpointRPM", intakeRPM);

    io.setVelocity(intakeRPM);
  }

  /** Returns the average velocity of the intake in rotations per minute (RPM). */
  @AutoLogOutput
  public double getIntakeVelocity() {
    return (inputs.bottomVelocityRPM + inputs.topVelocityRPM) / 2;
  }
}
