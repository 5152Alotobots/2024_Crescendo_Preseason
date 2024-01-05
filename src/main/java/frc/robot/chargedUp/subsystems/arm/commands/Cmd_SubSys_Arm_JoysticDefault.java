// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Robot;
import frc.robot.chargedup.subsystems.arm.SubSys_Arm;
import frc.robot.library.driverstation.JoystickUtilities;
import java.util.function.DoubleSupplier;

public class Cmd_SubSys_Arm_JoysticDefault extends CommandBase {
  /** Creates a new Cmd_SubSys_Arm_JoysticDefault. */
  SubSys_Arm subSys_Arm;

  private final DoubleSupplier armRotateAxis;
  private final DoubleSupplier armExtendAxis;

  public Cmd_SubSys_Arm_JoysticDefault(
      SubSys_Arm subSys_Arm, DoubleSupplier armRotateAxis, DoubleSupplier armExtendAxis) {

    this.subSys_Arm = subSys_Arm;
    this.armRotateAxis = armRotateAxis;
    this.armExtendAxis = armExtendAxis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subSys_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.subSys_Arm.setArmCmd(
        JoystickUtilities.joyDeadBndSqrdScaled(
            this.armRotateAxis.getAsDouble(), 0.05, Robot.Calibrations.Arm.ArmMaxRotSpd),
        JoystickUtilities.joyDeadBndSqrdScaled(
            this.armExtendAxis.getAsDouble(), 0.05, Robot.Calibrations.Arm.ArmExtensionMaxSpd));
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
