// File originally made by: Mechanical Advantage - FRC 6328
// Copyright (c) 2024 FRC 6328 (https://github.com/Mechanical-Advantage)
// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.SimConstants;

public class DriveIOSim implements DriveIO {
    // Built-in WPILib drivetrain simulation
    private DifferentialDrivetrainSim sim;

    // Create booleans to be used to simulate certain modes of the robot
    private boolean driveCoast = false;
    private boolean closedLoop = false;

    // Create limiters for coast mode slowdown
    private final SlewRateLimiter leftVoltsLimiter = new SlewRateLimiter(1.0);
    private final SlewRateLimiter rightVoltsLimiter = new SlewRateLimiter(1.0);

    // Store simulated drivetrain voltage values
    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;

    // Create PID controller for both sides (need to create some sort of sim pid values)
    private PIDController leftPID = new PIDController(SimConstants.kP, SimConstants.kI, SimConstants.kD);
    private PIDController rightPID = new PIDController(SimConstants.kP, SimConstants.kI, SimConstants.kD);

    // Create feedforward controller to calculate feedforward values
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SimConstants.kV, SimConstants.kS);
    
    // "Constructor" class, run when the class is first initialized
    public DriveIOSim(KitbotMotor simulatedMotors, KitbotGearing gearRatio, KitbotWheelSize wheelSize) {
        // I hate these Kitbot classes. There must be a better way to sync the real gear ratio and the sim gear ratio. I'll probably try to create a custom DifferentialDrivetrainSim class eventually.
        // Setup drivetrain simulation
        sim = DifferentialDrivetrainSim.createKitbotSim(simulatedMotors, gearRatio, wheelSize, null);
    }

    // In simulation, calling "updateInputs" also actually updates the simulation
    public void updateInputs(DriveIOInputs inputs) {
        // If we are in closed-loop control, run PID and feedforward calculations
        if (closedLoop) {
            // Probably should document these slightly better, not right now though
            leftAppliedVolts =
                MathUtil.clamp(
                    leftPID.calculate(sim.getLeftVelocityMetersPerSecond() / DriveConstants.metersPerRotation)
                        + feedforward.calculate(sim.getLeftVelocityMetersPerSecond() / DriveConstants.metersPerRotation),
                    -12.0,
                    12.0);
            rightAppliedVolts =
                MathUtil.clamp(
                    rightPID.calculate(sim.getRightVelocityMetersPerSecond() / DriveConstants.metersPerRotation)
                        + feedforward.calculate(sim.getRightVelocityMetersPerSecond() / DriveConstants.metersPerRotation),
                    -12.0,
                    12.0);

            // Send these values to the simulation
            sim.setInputs(leftAppliedVolts, rightAppliedVolts);
        }

        // If coast mode is enabled, run coast simulation
        if (driveCoast) {
            if (DriverStation.isDisabled()) {
                // Stop motors, but prevent the voltage from lowering too quickly
                setVoltage(leftVoltsLimiter.calculate(0.0), rightVoltsLimiter.calculate(0.0));
            } else {
                // Reset the value for the voltage limiter to be based off of
                leftVoltsLimiter.reset(leftAppliedVolts);
                rightVoltsLimiter.reset(rightAppliedVolts);
            }
        }

        // Update simulation with current values
        sim.update(Constants.loopPeriodSecs);

        // Set inputs for left drive side
        inputs.leftPositionRotations = sim.getLeftPositionMeters() / DriveConstants.metersPerRotation;
        inputs.leftVelocityRotationsPerSec = sim.getLeftVelocityMetersPerSecond() / DriveConstants.metersPerRotation;
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftCurrentAmps = new double[] {sim.getLeftCurrentDrawAmps()};

        // Set inputs for right drive side
        inputs.rightPositionRotations = sim.getRightPositionMeters() / DriveConstants.metersPerRotation;
        inputs.rightVelocityRotationsPerSec = sim.getRightVelocityMetersPerSecond() / DriveConstants.metersPerRotation;
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.rightCurrentAmps = new double[] {sim.getRightCurrentDrawAmps()};

        // Set inputs for gyro
        inputs.gyroYaw = sim.getHeading();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        // No longer running in closed loop, set closed loop to false
        closedLoop = false;

        // Ensure both values are in between -12.0 and 12.0
        leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
        rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);

        // Send values to the simulation
        sim.setInputs(leftAppliedVolts, rightAppliedVolts);
    }

    @Override
    public void setSpeed(double leftSpeed, double rightSpeed) {
        // Convert speed values to voltage and set the voltage
        setVoltage(leftSpeed * 12.0, rightSpeed * 12.0);
    }

    @Override
    public void setVelocity(double leftMetersPerSec, double rightMetersPerSec) {
        // Now running in closed loop, set closed loop to true
        closedLoop = true;
        
        // Set the setpoint of the PID loops
        leftPID.setSetpoint(leftMetersPerSec);
        rightPID.setSetpoint(rightMetersPerSec);
    }
}