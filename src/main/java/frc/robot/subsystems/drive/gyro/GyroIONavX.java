// Copyright (c) 2024 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
//
// Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

/** Drivetrain Gyro IO implementation for a Kauai Labs navX(2)-MXP on the SPI interface. */
public class GyroIONavX implements GyroIO {
  // Variable to store Gyro/AHRS object
  private AHRS gyro;

  public GyroIONavX() {
    try {
      // Try to create Gyro/AHRS object
      gyro = new AHRS(SPI.Port.kMXP);

      // Reset the gyro
      gyro.reset();
    } catch (RuntimeException ex) {
      System.out.print(
          "The Kauai Labs navX-MXP Gyro has failed to initialize! Unexpected behavior may happen!");
      DriverStation.reportWarning("Gyro failed to initialize!", false);
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Check to make sure the gyro is connected
    inputs.connected = gyro.isConnected();

    // Update normal odometry inputs
    inputs.yaw = Rotation2d.fromDegrees(gyro.getAngle());
    inputs.yawVelocityDegreesPerSecond = gyro.getRate();

    // Update the "queue" (would be used for >50hz odometry, but not used here, so just one value)
    inputs.odometryYawPositions = new Rotation2d[] {inputs.yaw};
  }

  @Override
  public void setYaw(double yaw) {
    // Find the offset needed to change the yaw to the specified value
    double offset = yaw - gyro.getYaw();
    // Set the angle offset
    gyro.setAngleAdjustment(offset);
  }
}
