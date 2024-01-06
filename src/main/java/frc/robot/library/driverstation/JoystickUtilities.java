/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.library.driverstation;

/** Add your docs here. */
public class JoystickUtilities {

  public static double joyDeadBnd(double rawJoy, double deadband) {
    double joy = 0;
    if ((rawJoy < deadband) && (rawJoy > deadband * (-1))) {
      joy = 0;
    } else {
      joy = (rawJoy - (Math.abs(rawJoy) / rawJoy * deadband)) / (1 - deadband);
    }
    return joy;
  }

  public static double joySqrd(double rawJoy) {
    double joy = 0;
    joy = rawJoy * (Math.abs(rawJoy));
    return joy;
  }

  public static double joyScaled(double rawJoy, double scalefactor) {
    double joy = 0;
    joy = rawJoy * scalefactor;
    return joy;
  }

  public static double joyDeadBndSqrd(double rawJoy, double deadband) {
    double joy = 0;
    joy = joySqrd(joyDeadBnd(rawJoy, deadband));
    return joy;
  }

  public static double joyDeadBndScaled(double rawJoy, double deadband, double scalefactor) {
    double joy = 0;
    joy = joyScaled(joyDeadBnd(rawJoy, deadband), scalefactor);
    return joy;
  }

  public static double joyDeadBndSqrdScaled(double rawJoy, double deadband, double scalefactor) {
    double joy = 0;
    joy = joyScaled(joyDeadBndSqrd(rawJoy, deadband), scalefactor);
    return joy;
  }
}
