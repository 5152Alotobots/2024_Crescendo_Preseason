// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
  _____                                                                _
 / ____|                      /\                                      | |
| (___   ___  __ _ _ __      /  \   _ __  _ __  _ __ _____   _____  __| |
 \___ \ / _ \/ _` | '_ \    / /\ \ | '_ \| '_ \| '__/ _ \ \ / / _ \/ _` |
 ____) |  __/ (_| | | | |  / ____ \| |_) | |_) | | | (_) \ V /  __/ (_| |
|_____/ \___|\__,_|_| |_| /_/    \_\ .__/| .__/|_|  \___/ \_/ \___|\__,_|
                                   | |   | |
                                   |_|   |_|
*/

package frc.robot.commands.auto.doubleelement;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.commands.Cmd_SubSys_Arm_RotateAndExtend;
import frc.robot.subsystems.arm.SubSys_Arm;
import frc.robot.subsystems.chargestation.commands.Cmd_SubSys_ChargeStation_Balance;
import frc.robot.subsystems.hand.SubSys_Hand;
import frc.robot.library.drivetrains.commands.Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj;
import frc.robot.library.drivetrains.SubSys_DriveTrain;
import frc.robot.library.gyroscopes.pigeon2.SubSys_PigeonGyro;

/** *Link For PathPlaner * */
public class Auto_middlebluecone_Cmd extends SequentialCommandGroup {
  private final SubSys_DriveTrain m_DriveTrain;
  private final SubSys_PigeonGyro m_pigeonGyro;
  private final SubSys_Arm m_Arm;
  private final SubSys_Hand m_Hand;

  /** Creates a new Auto_Challenge1_Cmd. */
  public Auto_middlebluecone_Cmd(
      SubSys_DriveTrain driveSubSys,
      SubSys_PigeonGyro pigeonGyro,
      SubSys_Arm subsysArm,
      SubSys_Hand subsysHand) {
    m_DriveTrain = driveSubSys;
    m_pigeonGyro = pigeonGyro;
    m_Arm = subsysArm;
    m_Hand = subsysHand;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new Cmd_whatever the arm one is
        // Hand is reversed
        new Cmd_SubSys_Arm_RotateAndExtend(subsysArm, -147.0, true, 1.54, true).withTimeout(4),
        new WaitCommand(2.5),
        new InstantCommand(subsysHand::CloseHand, subsysHand),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new Cmd_SubSys_Arm_RotateAndExtend(subsysArm, 0, false, 0.8, true).withTimeout(4),
                new Cmd_SubSys_Arm_RotateAndExtend(subsysArm, 42, true, 0.8, false).withTimeout(4),
                new Cmd_SubSys_Arm_RotateAndExtend(subsysArm, 42, false, 1, true).withTimeout(4)),
            new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(
                driveSubSys, "middlebluecone1", true, true, Alliance.Blue)),
        new InstantCommand(subsysHand::OpenHand, subsysHand),
        new ParallelCommandGroup(
            new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(
                driveSubSys, "middlebluecone2", false, false, Alliance.Blue),
            new Cmd_SubSys_Arm_RotateAndExtend(subsysArm, 10.0, true, 0.8, true).withTimeout(4)),
        new Cmd_SubSys_ChargeStation_Balance(pigeonGyro, driveSubSys));
  }
}
