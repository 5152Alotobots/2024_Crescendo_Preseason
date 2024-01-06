// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library.gyroscopes.navx;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */
public class SubSys_NavXGyro_Shuffleboard {

  private ShuffleboardTab m_ShflTab_NavXGyroSubSys;
  private NetworkTableEntry m_RawGyro;
  private NetworkTableEntry m_GyroAngleDeg;
  private NetworkTableEntry m_GyroRotation2dDeg;

  /** Constructor for NavXGyroSubSys Shuffleboard Tab */
  public SubSys_NavXGyro_Shuffleboard() {

    m_ShflTab_NavXGyroSubSys = Shuffleboard.getTab("NavXGyroSubSys");
    /*
          m_RawGyro = m_ShflTab_NavXGyroSubSys.add("RawGyro", 99)
          .withSize(6, 2)
          .withPosition(1, 3)
          .getEntry();

          m_GyroAngleDeg = m_ShflTab_NavXGyroSubSys.add("GyroAngleDeg", 99)
          .withSize(6, 2)
          .withPosition(1, 6)
          .getEntry();

          m_GyroRotation2dDeg = m_ShflTab_NavXGyroSubSys.add("GyroRotation2dDeg", 99)
          .withSize(6, 2)
          .withPosition(1, 9)
          .getEntry();
    */
  }

  /**
   * Drive SubSystem Shuffleboard Display
   *
   * @param rawGyroAngle Raw Gyro Angle
   * @param gyroAngle Gyro Angle Degrees
   * @param gyroRotation2d Rotation2d
   */
  public void Display_NavXGyroSubSys(
      double rawGyroAngle, double gyroAngle, Rotation2d gyroRotation2d) {

    // Update Drive Commands
    m_RawGyro.setDouble(rawGyroAngle);
    m_GyroAngleDeg.setDouble(gyroAngle);
    m_GyroRotation2dDeg.setDouble(gyroRotation2d.getDegrees());
  }
}
