/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.library.gyroscopes.navx;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubSys_NavXGyro extends SubsystemBase {
  /** Creates a new NavXGyro. */
  private AHRS m_NavXGyro = new AHRS(SPI.Port.kMXP);

  // NavXGyroSubSys Shuffleboard
  private SubSys_NavXGyro_Shuffleboard m_NavXGyroSubSys_Shuffleboard =
      new SubSys_NavXGyro_Shuffleboard();

  public SubSys_NavXGyro() {
    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      this.m_NavXGyro = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Display NavXGyro SubSys Shuffleboard
    if (SubSys_NavXGyro_Constants.NavXGyroSubSys_Shuffleboard_Enable) {
      m_NavXGyroSubSys_Shuffleboard.Display_NavXGyroSubSys(
          getRawGyroAngle(), getGyroAngle(), getGyroRotation2d());
    }
  }

  /**
   * Return Raw Gyro Angle
   *
   * @return Raw Angle double raw Gyro Angle
   */
  public double getRawGyroAngle() {
    return m_NavXGyro.getAngle();
  }

  /**
   * Return Gyro Angle in Degrees
   *
   * @return Gyro Angle double Degrees
   */
  public double getGyroAngle() {
    return Math.IEEEremainder(m_NavXGyro.getAngle(), 360)
        * (SubSys_NavXGyro_Constants.GyroReversed ? -1.0 : 1.0);
  }

  /**
   * Return Gyro Angle in Rotation2d
   *
   * @return Gyro Angle Rotation2d
   */
  public Rotation2d getGyroRotation2d() {
    return Rotation2d.fromDegrees(getGyroAngle());
  }

  /** Zero Gyro */
  public void zeroYaw() {
    m_NavXGyro.zeroYaw();
  }
}
