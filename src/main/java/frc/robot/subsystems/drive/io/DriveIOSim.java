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
import frc.robot.subsystems.drive.DriveConstants.SimConstants;

public class DriveIOSim implements DriveIO {
    // Built-in WPILib drivetrain simulation
    private DifferentialDrivetrainSim sim;

    // Wheel radius value that will be grabbed after setting up our simulation
    private double wheelRadius;

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
        // Setup drivetrain simulation
        sim = DifferentialDrivetrainSim.createKitbotSim(simulatedMotors, gearRatio, wheelSize, null);

        // Get the wheel radius value
        wheelRadius = wheelSize.value / 2;
    }

    // In simulation, calling "updateInputs" also actually updates the simulation
    public void updateInputs(DriveIOInputs inputs) {
        // If we are in closed-loop control, run PID and feedforward calculations
        if (closedLoop) {
            // Probably should document these slightly better, not right now though
            leftAppliedVolts =
                MathUtil.clamp(
                    leftPID.calculate(sim.getLeftVelocityMetersPerSecond() / wheelRadius)
                        + feedforward.calculate(sim.getLeftVelocityMetersPerSecond() / wheelRadius),
                    -12.0,
                    12.0);
            rightAppliedVolts =
                MathUtil.clamp(
                    rightPID.calculate(sim.getRightVelocityMetersPerSecond() / wheelRadius)
                        + feedforward.calculate(sim.getRightVelocityMetersPerSecond() / wheelRadius),
                    -12.0,
                    12.0);

            // Send these values to the simulation
            sim.setInputs(leftAppliedVolts, rightAppliedVolts)
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
        inputs.leftPositionRotations = sim.getLeftPositionMeters() / (2 * Math.PI * wheelRadius);

    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
    }

    @Override
    public void setVelocity(double leftRotPerSec, double rightRotPerSec) {
        leftPID.setSetpoint(leftRotPerSec * 60);
        rightPID.setSetpoint(rightRotPerSec * 60);
    }
}