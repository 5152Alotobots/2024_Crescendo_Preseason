// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library.motorcontrollers.talonsrx;

public class TalonSRX_Conversions {

  /******* Talon SRX with MA3 Analog Encoder *******/
  /**
   * @param counts MA3 Counts
   * @param gearRatio Gear Ratio between MA3 and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  public static double ma3ToDegrees(double counts, double gearRatio) {
    return counts * (360.0 / (gearRatio * 1024.0));
  }

  /**
   * @param degrees Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between MA3 and Mechanism
   * @return Falcon Counts
   */
  public static double degreesToMA3(double degrees, double gearRatio) {
    double ticks = degrees / (360.0 / (gearRatio * 1024.0));
    return ticks;
  }

  /**
   * @param velocityCounts MA3 Velocity Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double ma3ToRPM(double velocityCounts, double gearRatio) {
    double motorRPM = velocityCounts * (600.0 / 1024.0);
    double mechRPM = motorRPM / gearRatio;
    return mechRPM;
  }

  /**
   * @param RPM RPM of mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double rpmToMA3(double RPM, double gearRatio) {
    double motorRPM = RPM * gearRatio;
    double sensorCounts = motorRPM * (1024.0 / 600.0);
    return sensorCounts;
  }

  /**
   * @param velocitycounts MA3 Velocity Counts
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between MA3 and Mechanism (set to 1 for MA3 RPM)
   * @return MA3 Velocity Counts
   */
  public static double ma3ToMPS(double velocitycounts, double circumference, double gearRatio) {
    double wheelRPM = ma3ToRPM(velocitycounts, gearRatio);
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;
  }

  /**
   * @param velocity Velocity MPS
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   */
  public static double mpsToMA3(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    double wheelVelocity = rpmToMA3(wheelRPM, gearRatio);
    return wheelVelocity;
  }
}
