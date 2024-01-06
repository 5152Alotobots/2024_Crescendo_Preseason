/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.library.drivetrains.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Robot;
import frc.robot.library.drivetrains.SubSys_DriveTrain;
import frc.robot.library.driverstation.JoystickUtilities;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Cmd_SubSys_DriveTrain_JoysticDefault extends CommandBase {
  /** Creates a new FalconTalonFXDriveTalonSR. */
  private final SubSys_DriveTrain driveSubSys;

  private final DoubleSupplier fwdCmd;
  private final DoubleSupplier strCmd;
  private final DoubleSupplier rotCmd;
  private final boolean fieldOriented;
  private final BooleanSupplier rotateLeftPt;
  private final BooleanSupplier rotateRightPt;
  private final BooleanSupplier perfModeAActive; // Mode 1
  private final BooleanSupplier perfModeBActive; // Mode 2
  private double maxSpd = 0;
  private double maxRotSpd = 0;

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
   * @param perfModeAActive
   * @param perfModeBActive
   */
  public Cmd_SubSys_DriveTrain_JoysticDefault(
      SubSys_DriveTrain driveSubSys,
      DoubleSupplier fwdCmd,
      DoubleSupplier strCmd,
      DoubleSupplier rotCmd,
      boolean fieldOriented,
      BooleanSupplier rotateLeftPt,
      BooleanSupplier rotateRightPt,
      BooleanSupplier perfModeAActive,
      BooleanSupplier perfModeBActive) {

    this.driveSubSys = driveSubSys;
    this.fwdCmd = fwdCmd;
    this.strCmd = strCmd;
    this.rotCmd = rotCmd;
    this.fieldOriented = fieldOriented;
    this.rotateLeftPt = rotateLeftPt;
    this.rotateRightPt = rotateRightPt;
    this.perfModeAActive = perfModeAActive;
    this.perfModeBActive = perfModeBActive;
    addRequirements(driveSubSys);

    this.maxSpd = Robot.Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd;
    this.maxRotSpd = Robot.Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (perfModeAActive.getAsBoolean()) {
      this.maxSpd = Robot.Calibrations.DriveTrain.PerformanceMode_A.DriveTrainMaxSpd;
      this.maxRotSpd = Robot.Calibrations.DriveTrain.PerformanceMode_A.DriveTrainMaxRotSpd;
    } else if (perfModeBActive.getAsBoolean()) {
      this.maxSpd = Robot.Calibrations.DriveTrain.PerformanceMode_B.DriveTrainMaxSpd;
      this.maxRotSpd = Robot.Calibrations.DriveTrain.PerformanceMode_B.DriveTrainMaxRotSpd;
    } else {
      this.maxSpd = Robot.Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxSpd;
      this.maxRotSpd = Robot.Calibrations.DriveTrain.PerformanceMode_Default.DriveTrainMaxRotSpd;
    }

    driveSubSys.Drive(
        JoystickUtilities.joyDeadBndSqrdScaled(fwdCmd.getAsDouble(), 0.05, maxSpd),
        JoystickUtilities.joyDeadBndSqrdScaled(strCmd.getAsDouble(), 0.05, maxSpd),
        JoystickUtilities.joyDeadBndScaled(rotCmd.getAsDouble(), 0.1, maxRotSpd),
        fieldOriented,
        rotateLeftPt.getAsBoolean(),
        rotateRightPt.getAsBoolean());

    // SmartDashboard.putBoolean("RotateLeft_JoyCmd", m_RotateLeftPt.getAsBoolean());
    // SmartDashboard.putBoolean("RotateRight_JoyCmd", m_RotateRightPt.getAsBoolean());
    SmartDashboard.putBoolean("PerfModeA_Active", perfModeAActive.getAsBoolean());
    SmartDashboard.putBoolean("PerfModeB_Active", perfModeBActive.getAsBoolean());
    SmartDashboard.putNumber("MaxSpd", maxSpd);
    SmartDashboard.putNumber("MaxRotSpd", maxRotSpd);

    SmartDashboard.putNumber(
        "FwdCmd", JoystickUtilities.joyDeadBndSqrdScaled(fwdCmd.getAsDouble(), 0.05, maxSpd));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
