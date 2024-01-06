/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.library.drivetrains.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.library.drivetrains.SubSys_DriveTrain;
import frc.robot.library.drivetrains.SubSys_DriveTrain_Constants;
import frc.robot.library.driverstation.JoystickUtilities;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Cmd_SubSys_DriveTrain_JoysticTurbo extends CommandBase {
  /** Creates a new FalconTalonFXDriveTalonSR. */
  private final SubSys_DriveTrain driveSubSys;

  private final DoubleSupplier fwdCmd;
  private final DoubleSupplier strCmd;
  private final DoubleSupplier rotCmd;
  private final boolean fieldOriented;
  private final BooleanSupplier rotateLeftPt;
  private final BooleanSupplier rotateRightPt;

  /**
   * Cmd_SubSys_DriveTrain_JoysticDefault Joystick Drive Command
   *
   * @param driveSubSys
   * @param fwdCmd
   * @param strCmd
   * @param rotCmd
   * @param fieldOriented
   * @param rotateLeftPt
   * @param rotateRightPt
   */
  public Cmd_SubSys_DriveTrain_JoysticTurbo(
      SubSys_DriveTrain driveSubSys,
      DoubleSupplier fwdCmd,
      DoubleSupplier strCmd,
      DoubleSupplier rotCmd,
      boolean fieldOriented,
      BooleanSupplier rotateLeftPt,
      BooleanSupplier rotateRightPt) {

    this.driveSubSys = driveSubSys;
    this.fwdCmd = fwdCmd;
    this.strCmd = strCmd;
    this.rotCmd = rotCmd;
    this.fieldOriented = fieldOriented;
    this.rotateLeftPt = rotateLeftPt;
    this.rotateRightPt = rotateRightPt;
    addRequirements(driveSubSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ! DON'T DO THIS. CHANGE THIS AFTER THE COMP
    SubSys_DriveTrain_Constants.DriveTrainMaxPctOutput =
        SubSys_DriveTrain_Constants.DriveTrainMaxTurboPctOutput;
    SubSys_DriveTrain_Constants.DriveTrainMaxSpd =
        SubSys_DriveTrain_Constants.DriveTrainMaxTurboSpd;
    SubSys_DriveTrain_Constants.DriveTrainMaxAccel =
        SubSys_DriveTrain_Constants.DriveTrainMaxTurboAccel;

    SubSys_DriveTrain_Constants.DriveTrainMaxRotPctOutput =
        SubSys_DriveTrain_Constants.DriveTrainMaxTurboRotPctOutput;
    SubSys_DriveTrain_Constants.DriveTrainMaxRotSpeed =
        SubSys_DriveTrain_Constants.DriveTrainMaxTurboRotSpeed;
    SubSys_DriveTrain_Constants.DriveTrainMaxRotAccel =
        SubSys_DriveTrain_Constants.DriveTrainMaxTurboRotAccel;

    driveSubSys.Drive(
        JoystickUtilities.joyDeadBndSqrdScaled(
            fwdCmd.getAsDouble(), 0.05, driveSubSys.getMaxDriveTurboSubSysSpd()),
        JoystickUtilities.joyDeadBndSqrdScaled(
            strCmd.getAsDouble(), 0.05, driveSubSys.getMaxDriveTurboSubSysSpd()),
        // JoystickUtilities.joyDeadBndScaled(fwdCmd.getAsDouble(),0.1,m_DriveSubSys.getMaxDriveSubSysSpd()),
        // JoystickUtilities.joyDeadBndScaled(m_StrCmd.getAsDouble(),0.1,m_DriveSubSys.getMaxDriveSubSysSpd()),
        JoystickUtilities.joyDeadBndScaled(
            rotCmd.getAsDouble(), 0.1, driveSubSys.getMaxDriveSubSysTurboRotSpd()),
        fieldOriented,
        rotateLeftPt.getAsBoolean(),
        rotateRightPt.getAsBoolean());

    // SmartDashboard.putBoolean("RotateLeft_JoyCmd", m_RotateLeftPt.getAsBoolean());
    // SmartDashboard.putBoolean("RotateRight_JoyCmd", m_RotateRightPt.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // ! DON'T DO THIS. CHANGE THIS AFTER THE COMP
    SubSys_DriveTrain_Constants.DriveTrainMaxPctOutput =
        SubSys_DriveTrain_Constants.DriveTrainMaxDefaultPctOutput;
    SubSys_DriveTrain_Constants.DriveTrainMaxSpd =
        SubSys_DriveTrain_Constants.DriveTrainMaxDefaultSpd;
    SubSys_DriveTrain_Constants.DriveTrainMaxAccel =
        SubSys_DriveTrain_Constants.DriveTrainMaxDefaultAccel;

    SubSys_DriveTrain_Constants.DriveTrainMaxPctOutput =
        SubSys_DriveTrain_Constants.DriveTrainMaxDefaultPctOutput;
    SubSys_DriveTrain_Constants.DriveTrainMaxRotSpeed =
        SubSys_DriveTrain_Constants.DriveTrainMaxDefaultRotSpeed;
    SubSys_DriveTrain_Constants.DriveTrainMaxRotAccel =
        SubSys_DriveTrain_Constants.DriveTrainMaxDefaultRotAccel;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
