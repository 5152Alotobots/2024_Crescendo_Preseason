// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup.subsystems.bling.commands;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.chargedup.subsystems.bling.SubSys_Bling;
import frc.robot.chargedup.subsystems.bling.SubSys_Bling_Constants.Patterns;
import frc.robot.chargedup.subsystems.bling.SubSys_Bling_Constants.SolidColors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CmdGrp_SubSys_IdleColorSequence extends SequentialCommandGroup {
  private SubSys_Bling subSys_Bling;
  private Spark controller;

  /** Creates a new CmdGrp_IdleBlingColorSequence. */
  public CmdGrp_SubSys_IdleColorSequence(SubSys_Bling subSys_Bling, Spark Controller) {
    this.subSys_Bling = subSys_Bling;
    this.controller = Controller;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new Cmd_SubSys_Bling_SetColorValue(subSys_Bling, controller, Patterns.FixedPalette.StrobeRed),
        new WaitCommand(5),
        new Cmd_SubSys_Bling_SetColorValue(subSys_Bling, controller, SolidColors.DarkRed));
  }
}
