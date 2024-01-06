// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library.drivetrains.swervedrive;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class SwerveRotatePointLogic {
  /**
   * Logic to rotate about a point other than the robot center
   *
   * @param rotateLeftPtCmd Boolean Rotate about Left Point Command
   * @param rotateRightPtCmd Boolean Rotate about Right Point Command
   * @param rotateLeftPtCmd_prev Boolean Previous Rotate about Left Point Command
   * @param rotateRightPtCmd_prev Boolean Previous Rotate about Left Point Command
   * @param fieldRelative Boolean Field Relative
   * @param robotHeading Double Robot Heading in Degrees
   * @param xVelCmd Double Robot Forward Velocity Command
   * @return
   */
  public static Translation2d calcRotationPt(
      boolean rotateLeftPtCmd,
      boolean rotateRightPtCmd,
      boolean rotateLeftPtCmd_prev,
      boolean rotateRightPtCmd_prev,
      boolean fieldRelative,
      double robotHeading,
      double xVelCmd) {

    Translation2d rotationPt = new Translation2d(0, 0);

    // If Rotate Point Buttons are different than previous commands, create new rotation point.
    if ((rotateLeftPtCmd != rotateLeftPtCmd_prev) || (rotateRightPtCmd != rotateRightPtCmd_prev)) {

      // Check for only Rotate Left Cmd
      if (rotateLeftPtCmd && !rotateRightPtCmd) {
        if (fieldRelative) {
          // Facing Forward (Downfield) ~ 350-360 degrees (Use Non-Field Relative logic)
          if (robotHeading >= (360 - SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            if (xVelCmd >= 0) {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtFL;
            } else {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtBL;
            }
            // Facing Forward Right ~ 280-350 degrees
          } else if (robotHeading
              >= (270 + SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            rotationPt = SubSys_SwerveDrive_Constants.RotationPtBL;
            // Facing Right ~ 260-280 degrees
          } else if (robotHeading
              >= (270 - SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            if (xVelCmd >= 0) {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtBL;
            } else {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtBR;
            }
            // Facing Rear Right ~ 190-260 degrees
          } else if (robotHeading
              >= (180 + SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            rotationPt = SubSys_SwerveDrive_Constants.RotationPtBR;
            // Facing Rear ~ 170-190 degrees
          } else if (robotHeading
              >= (180 - SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            if (xVelCmd >= 0) {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtBR;
            } else {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtFL;
            }
            // Facing Rear Left ~ 100-170 degrees
          } else if (robotHeading
              >= (90 + SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            rotationPt = SubSys_SwerveDrive_Constants.RotationPtFR;
            // Facing Left ~ 80-100 degrees
          } else if (robotHeading
              >= (90 - SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            if (xVelCmd >= 0) {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtFR;
            } else {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtFL;
            }
            // Facing Forward Left ~ 10-80 degrees
          } else if (robotHeading >= (SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            rotationPt = SubSys_SwerveDrive_Constants.RotationPtFL;
            // Facing Forward ~ 0-10 degrees
          } else {
            if (xVelCmd >= 0) {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtFL;
            } else {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtBL;
            }
          }
          // Not Field Relative
        } else {
          if (xVelCmd >= 0) {
            rotationPt = SubSys_SwerveDrive_Constants.RotationPtFL;
          } else {
            rotationPt = SubSys_SwerveDrive_Constants.RotationPtBL;
          }
        }

        // Check for only Rotate Right Cmd
      } else if (!rotateLeftPtCmd && rotateRightPtCmd) {
        if (fieldRelative) {
          // Facing Forward (Downfield) ~ 350-360 degrees (Use Non-Field Relative logic)
          if (robotHeading >= (360 - SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            if (xVelCmd >= 0) {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtFR;
            } else {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtBR;
            }
            // Facing Forward Right ~ 280-350 degrees
          } else if (robotHeading
              >= (270 + SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            rotationPt = SubSys_SwerveDrive_Constants.RotationPtFR;
            // Facing Right ~ 260-280 degrees
          } else if (robotHeading
              >= (270 - SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            if (xVelCmd >= 0) {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtFR;
            } else {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtBR;
            }
            // Facing Rear Right ~ 190-260 degrees
          } else if (robotHeading
              >= (180 + SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            rotationPt = SubSys_SwerveDrive_Constants.RotationPtFL;
            // Facing Rear ~ 170-190 degrees
          } else if (robotHeading
              >= (180 - SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            if (xVelCmd >= 0) {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtBL;
            } else {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtFL;
            }
            // Facing Rear Left ~ 100-170 degrees
          } else if (robotHeading
              >= (90 + SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            rotationPt = SubSys_SwerveDrive_Constants.RotationPtBL;
            // Facing Left ~ 80-100 degrees
          } else if (robotHeading
              >= (90 - SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            if (xVelCmd >= 0) {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtBR;
            } else {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtBL;
            }
            // Facing Forward Left ~ 10-80 degrees
          } else if (robotHeading >= (SubSys_SwerveDrive_Constants.RotateFieldRelativeMargin)) {
            rotationPt = SubSys_SwerveDrive_Constants.RotationPtBR;
            // Facing Forward ~ 0-10 degrees
          } else {
            if (xVelCmd >= 0) {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtFR;
            } else {
              rotationPt = SubSys_SwerveDrive_Constants.RotationPtBR;
            }
          }
        } else {
          if (xVelCmd >= 0) {
            rotationPt = SubSys_SwerveDrive_Constants.RotationPtFR;
          } else {
            rotationPt = SubSys_SwerveDrive_Constants.RotationPtBR;
          }
        }
        // If not Rotate Pt Cmds or BOTH Rotate Pt Cmds set Rotation Point to Center
      } else {
        rotationPt = new Translation2d(0, 0);
      }
    }
    return rotationPt;
  }
}
