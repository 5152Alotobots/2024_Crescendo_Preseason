// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.SubSys_Arm;

public class Cmd_SubSys_Arm_RotateAndExtend extends CommandBase {
  /** Creates a new Cmd_SubSys_Arm_JoysticDefault. */
  SubSys_Arm subSys_Arm;

  private final double armRotatePosCmd;
  private final boolean armRotateEnable;
  private final double armExtendPosCmd;
  private final boolean armExtendEnable;
  private boolean finished;

  public Cmd_SubSys_Arm_RotateAndExtend(
      SubSys_Arm subSys_Arm,
      double armRotatePosCmd,
      boolean armRotateEnable,
      double armExtendPosCmd,
      boolean armExtendEnable) {

    this.subSys_Arm = subSys_Arm;
    this.armRotatePosCmd = armRotatePosCmd;
    this.armRotateEnable = armRotateEnable;
    this.armExtendPosCmd = armExtendPosCmd;
    this.armExtendEnable = armExtendEnable;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subSys_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("ArmRotatePosCmd", this.armRotatePosCmd);
    this.finished =
        this.subSys_Arm.setArmPosCmd(
            this.armRotatePosCmd, this.armRotateEnable, this.armExtendPosCmd, this.armExtendEnable);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.subSys_Arm.setArmCmd(0, 0);
    SmartDashboard.putNumber("ArmRotatePosCmd", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.finished;
    // return false;
  }
}
