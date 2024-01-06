// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
__          __           _                                      _          _                       _
\ \        / /          | |                          /\        | |        | |                     | |
 \ \  /\  / /_ _ _   _  | |_ ___     __ _  ___      /  \  _   _| |_ ___   | |_ ___  __ _ _ __ ___ | |
  \ \/  \/ / _` | | | | | __/ _ \   / _` |/ _ \    / /\ \| | | | __/ _ \  | __/ _ \/ _` | '_ ` _ \| |
   \  /\  / (_| | |_| | | || (_) | | (_| | (_) |  / ____ \ |_| | || (_) | | ||  __/ (_| | | | | | |_|
    \/  \/ \__,_|\__, |  \__\___/   \__, |\___/  /_/    \_\__,_|\__\___/   \__\___|\__,_|_| |_| |_(_)
                  __/ |              __/ |
                 |___/              |___/
*/
package frc.robot.commands.auto.singleelement.cube;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.commands.Cmd_SubSys_Arm_RotateAndExtend;
import frc.robot.subsystems.arm.SubSys_Arm;
import frc.robot.subsystems.bling.commands.Cmd_SubSys_Bling_SetColorValue;
import frc.robot.subsystems.bling.SubSys_Bling_Constants;
import frc.robot.subsystems.bling.SubSys_Bling;
import frc.robot.library.vision.photonvision.commands.Cmd_NavigateToBestVisionTarget;
import frc.robot.subsystems.hand.SubSys_Hand;
import frc.robot.library.vision.photonvision.SubSys_Photonvision_Constants;
import frc.robot.library.vision.photonvision.SubSys_Photonvision;
import frc.robot.library.drivetrains.commands.Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj;
import frc.robot.library.drivetrains.SubSys_DriveTrain;
import frc.robot.library.gyroscopes.pigeon2.SubSys_PigeonGyro;

// For information see slides linked below
// Link For PathPlaner
// https://docs.google.com/presentation/d/1xjYSI4KpbmGBUY-ZMf1nAFrXIoJo1tl-HHNl8LLqa1I/edit#slide=id.g1e64fa08ff8_0_0

public class Auto_vision_blue_Cmd extends SequentialCommandGroup {
  private final SubSys_DriveTrain m_DriveTrain;
  private final SubSys_PigeonGyro m_pigeonGyro;
  private final SubSys_Arm subsysArm;
  private final SubSys_Hand subsysHand;
  private final SubSys_Bling blingSubSys;
  private final SubSys_Photonvision photonvision;

  /** Creates a new Auto_Challenge1_Cmd. */
  public Auto_vision_blue_Cmd(
      SubSys_DriveTrain driveSubSys,
      SubSys_Arm arm,
      SubSys_Hand hand,
      SubSys_PigeonGyro pigeonGyro,
      SubSys_Bling bling,
      SubSys_Photonvision photonvision) {
    m_DriveTrain = driveSubSys;
    m_pigeonGyro = pigeonGyro;
    subsysArm = arm;
    subsysHand = hand;
    blingSubSys = bling;
    this.photonvision = photonvision;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    /* Construct parallel command groups */
    ParallelCommandGroup driveAndMoveToPickupPosition =
        new ParallelCommandGroup(
            new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(
                driveSubSys, "vision1", true, true, Alliance.Blue),
            new Cmd_SubSys_Arm_RotateAndExtend(subsysArm, 0.0, true, 0.8, true));
    ParallelCommandGroup driveAndDeliverCone =
        new ParallelCommandGroup(
            new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(
                driveSubSys, "vision2", false, false, Alliance.Blue),
            new SequentialCommandGroup(
                new Cmd_SubSys_Arm_RotateAndExtend(subsysArm, -90, true, 0, false).withTimeout(4),
                new Cmd_SubSys_Arm_RotateAndExtend(subsysArm, -158.0, true, 1.54, true).withTimeout(6)));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new Cmd_SubSys_Arm_RotateAndExtend(subsysArm, -100.0, true, .85, false)
            .withTimeout(1.5), // Lift arm to high position
        new Cmd_SubSys_Arm_RotateAndExtend(subsysArm, -145.0, true, 1.54, true)
            .withTimeout(3), // Lift arm to high position
        new WaitCommand(1), // Add buffer time
        new InstantCommand(subsysHand::CloseHand, subsysHand), // Open hand (reversed)
        driveAndMoveToPickupPosition, // Drive to end position
        new Cmd_NavigateToBestVisionTarget(
                driveSubSys,
                photonvision,
                bling,
                SubSys_Photonvision_Constants.Cameras.frontCamera,
                SubSys_Photonvision_Constants.Pipelines.Cube,
                true,
                false,
                true)
            .withTimeout(2.5),
        new Cmd_SubSys_Arm_RotateAndExtend(subsysArm, 40, true, 0.8, true),
        new InstantCommand(subsysHand::OpenHand, subsysHand), // Close hand (reversed)
        driveAndDeliverCone,
        new InstantCommand(subsysHand::CloseHand, subsysHand), // Open hand (reversed)
        new Cmd_SubSys_Arm_RotateAndExtend(subsysArm, 45, true, 0, true).withTimeout(4),
        new Cmd_SubSys_Bling_SetColorValue(
            blingSubSys,
            SubSys_Bling_Constants.Controllers.controller1,
            SubSys_Bling_Constants.Patterns.FixedPalette.RainbowRainbow) // Celebrate!
        );
  }
}
