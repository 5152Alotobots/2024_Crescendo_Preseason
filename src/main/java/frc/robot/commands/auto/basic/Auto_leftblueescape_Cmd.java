// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.basic;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.library.drivetrains.commands.Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj;
import frc.robot.library.drivetrains.SubSys_DriveTrain;
import frc.robot.library.gyroscopes.pigeon2.SubSys_PigeonGyro;

// For information see slides linked below
// Link For PathPlaner
// https://docs.google.com/presentation/d/1xjYSI4KpbmGBUY-ZMf1nAFrXIoJo1tl-HHNl8LLqa1I/edit#slide=id.g1e64fa08ff8_0_0

public class Auto_leftblueescape_Cmd extends SequentialCommandGroup {
  private final SubSys_DriveTrain m_DriveTrain;
  private final SubSys_PigeonGyro m_pigeonGyro;

  /** Creates a new Auto_Challenge1_Cmd. */
  public Auto_leftblueescape_Cmd(SubSys_DriveTrain driveSubSys, SubSys_PigeonGyro pigeonGyro) {
    m_DriveTrain = driveSubSys;
    m_pigeonGyro = pigeonGyro;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(
            driveSubSys, "leftblueescape", true, true, Alliance.Blue));
  }
}
