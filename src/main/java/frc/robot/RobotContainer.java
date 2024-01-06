/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.commands.Cmd_SubSys_Arm_JoysticDefault;
import frc.robot.subsystems.arm.commands.Cmd_SubSys_Arm_RotateAndExtend;
import frc.robot.subsystems.arm.SubSys_Arm;
import frc.robot.subsystems.bling.commands.Cmd_SubSys_Bling_SetColorValue;
import frc.robot.subsystems.bling.SubSys_Bling_Constants;
import frc.robot.subsystems.bling.SubSys_Bling;
import frc.robot.subsystems.hand.SubSys_Hand;
import frc.robot.library.vision.photonvision.SubSys_Photonvision;
import frc.robot.library.drivetrains.commands.Cmd_SubSys_DriveTrain_JoysticDefault;
import frc.robot.library.drivetrains.SubSys_DriveTrain;
import frc.robot.library.gyroscopes.pigeon2.SubSys_PigeonGyro;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SubSys_PigeonGyro gyroSubSys = new SubSys_PigeonGyro();
  private final SubSys_DriveTrain driveSubSys = new SubSys_DriveTrain(gyroSubSys);
  private final SubSys_Hand handSubSys = new SubSys_Hand();
  private final SubSys_Arm armSubSys = new SubSys_Arm(handSubSys.getHandLength());
  private final SubSys_Bling blingSubSys = new SubSys_Bling();
  private final DriverStation driverStation = new DriverStation();
  private final Auto auto;

  public RobotContainer() {
    SubSys_Photonvision photonvisionSubSys = new SubSys_Photonvision();

    auto = new Auto(blingSubSys, photonvisionSubSys, handSubSys, armSubSys, gyroSubSys, driveSubSys);

    configureButtonBindings();

    armSubSys.setDefaultCommand(
        new Cmd_SubSys_Arm_JoysticDefault(
            armSubSys,
                driverStation::GetArmRotateAxis,
                driverStation::GetArmExtendAxis));

    driveSubSys.setDefaultCommand(
        new Cmd_SubSys_DriveTrain_JoysticDefault(
            driveSubSys,
                driverStation::DriveFwdAxis,
                driverStation::DriveStrAxis,
                driverStation::DriveRotAxis,
            true,
                driverStation::RotateLeftPt,
                driverStation::RotateRightPt,
                driverStation::DrivePerfModeAActive,
                driverStation::DrivePerfModeBActive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}. Use this method to define your
   * button->command mappings. Buttons can be created by instantiating a {@link GenericHID} or one
   * of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Gyro Reset Command Button
    driverStation.OpenHandButton.onTrue(new InstantCommand(handSubSys::OpenHand, handSubSys));
    driverStation.CloseHandButton.onTrue(
        new InstantCommand(handSubSys::CloseHand, handSubSys));
    driverStation.GyroResetButton.onTrue(new InstantCommand(gyroSubSys::zeroYawToFront, gyroSubSys));

    // Gyro Reset Command Button
    driverStation.PoseResetButton.onTrue(
        // new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));
        new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));

    // Test Button
    driverStation.TestButton.whileTrue(
        new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, -145.0, true, 1.54, true)
        //   new CmdGrp_TestVisionAuto(driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys,
        // photonvisionSubSys)
        );

    driverStation.GroundPickupButton.whileTrue(
        new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, 45.0, true, 0.8, true));

    driverStation.HighConeDelivery.whileTrue(
        new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, -35.0, true, 1.65, true));

    driverStation.MidConeDelivery.whileTrue(
        new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, -25.0, true, 1.00, true));

    driverStation.HighSafePos.whileTrue(
        new Cmd_SubSys_Arm_RotateAndExtend(armSubSys, -80.0, true, 0.8, true));

    // CONE/CUBE SIGNALING
    driverStation.RequestConeButton.onTrue(
        new Cmd_SubSys_Bling_SetColorValue(
            blingSubSys, SubSys_Bling_Constants.Controllers.controller1, SubSys_Bling_Constants.SolidColors.Yellow));
    driverStation.RequestCubeButton.onTrue(
        new Cmd_SubSys_Bling_SetColorValue(
            blingSubSys, SubSys_Bling_Constants.Controllers.controller1, SubSys_Bling_Constants.SolidColors.Violet));

    // Fun signaling
    driverStation.ResetLEDColorButton.onTrue(
        new Cmd_SubSys_Bling_SetColorValue(
            blingSubSys,
            SubSys_Bling_Constants.Controllers.controller1,
            SubSys_Bling_Constants.Patterns.Color1Color2.ColorWaves));
    driverStation.RainbowLEDColorButton.onTrue(
        new Cmd_SubSys_Bling_SetColorValue(
            blingSubSys,
            SubSys_Bling_Constants.Controllers.controller1,
            SubSys_Bling_Constants.Patterns.FixedPalette.RainbowRainbow));
    driverStation.RainbowStrobeLEDColorButton.onTrue(
        new Cmd_SubSys_Bling_SetColorValue(
            blingSubSys,
            SubSys_Bling_Constants.Controllers.controller1,
            SubSys_Bling_Constants.Patterns.FixedPalette.StrobeRed));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto.getAutoCommand();
  }
}
