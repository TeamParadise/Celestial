// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.io;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import frc.robot.subsystems.drive.DriveConstants;

public class DriveIOSim implements DriveIO {
    private DifferentialDrivetrainSim sim;

    // Create booleans to be used to simulate certain modes of the robot
    private boolean driveCoast = false;
    private boolean closedLoop = false;

    // Create PID controller for both sides (need to create some sort of sim pid values)
    private PIDController leftPID = new PIDController(0, 0, 0);
    private PIDController rightPID = new PIDController(0, 0, 0);
    
    // "Constructor" class, run when the class is first initialized
    public void DriveIOSim(KitbotMotor simulatedMotors, KitbotGearing gearRatio, KitbotWheelSize wheelSize) {
        sim = DifferentialDrivetrainSim.createKitbotSim(simulatedMotors, gearRatio, wheelSize, null);
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