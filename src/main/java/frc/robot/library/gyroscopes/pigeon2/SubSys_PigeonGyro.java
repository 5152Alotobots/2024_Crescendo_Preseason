/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.library.gyroscopes.pigeon2;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.library.gyroscopes.navx.SubSys_NavXGyro_Constants;

public class SubSys_PigeonGyro extends SubsystemBase {
  /** Creates a new NavXGyro. */
  private final Pigeon2 pigeon2Gyro;

  private double gyroOffsetDegrees;

  public SubSys_PigeonGyro() {
    this.pigeon2Gyro = new Pigeon2(Constants.CAN_IDs.Pigeon2_ID);
    this.gyroOffsetDegrees = 0.0;
    // m_Pigeon2Gyro.setYaw(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Display
    SmartDashboard.putNumber("Pigeon_getRawYaw", getRawYaw());
    SmartDashboard.putNumber("Pigeon_getYaw", getYaw());
    SmartDashboard.putNumber("Pigeon_getYawRotation2d", getYawRotation2d().getDegrees());
    // SmartDashboard.putNumber("GyroCompass", m_Pigeon2Gyro.getCompassHeading());
  }

  /**
   * Return the raw gyro pitch
   * @info In default orientation (z to the sky, x forward, y to the left) pitch is a rotation around the y (front to back tilt)
   * @return gyro pitch degrees (as a double)
   */
  public double getRawGyroPitch() {
    return this.pigeon2Gyro.getPitch();
  }

  /**
   * Return the raw gyro roll
   * @info In default orientation (z to the sky, x forward, y to the left) Roll is a rotation around the x (side to side tilt)
   * @return gyro roll degrees (as a double)
   */
  public double getRawGyroRoll() {
    return this.pigeon2Gyro.getRoll();
  }

  /**
   * Return the raw gyro yaw
   * @info In default orientation (z to the sky, x forward, y to the left) Yaw is a rotation around the z (rotation)
   * @return gyro yaw degrees (as a double)
   */
  public double getRawYaw() {
    return this.pigeon2Gyro.getYaw();
  }

  /**
   * @return Yaw accounting for a set Offset point
   */
  public double getYaw() {
    return getRawYaw() - this.gyroOffsetDegrees;
  }

  public void zeroYawToFront() {
    this.gyroOffsetDegrees = getRawYaw();
  }

  /** Set Gyro */
  public void setYawRelativeToFront(double yawDegrees) {
    this.gyroOffsetDegrees = getRawYaw() - yawDegrees;
  }

  /**
   * Return Gyro Angle in Degrees
   *
   * @return Gyro Angle double Degrees
   */
  public double getYawAngle() {
    return Math.IEEEremainder(getYaw(), 360)
        * (SubSys_NavXGyro_Constants.GyroReversed ? -1.0 : 1.0);
  }

  /**
   * Return Gyro Angle in Rotation2d
   *
   * @return Gyro Angle Rotation2d
   */
  public Rotation2d getYawRotation2d() {
    return Rotation2d.fromDegrees(getYawAngle());
  }
}
