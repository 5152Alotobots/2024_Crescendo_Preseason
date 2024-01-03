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
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_JoysticDefault;
import frc.robot.ChargedUp.Arm.Cmds_SubSys_Arm.Cmd_SubSys_Arm_PosCmd;
import frc.robot.ChargedUp.Bling.Const_Bling;
import frc.robot.ChargedUp.Bling.SubSys_Bling;
import frc.robot.ChargedUp.Bling.Cmd.Cmd_SetBlingColorValue;
import frc.robot.ChargedUp.DriverStation.SubSys_DriverStation;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.ChargedUp.PhotonVision.SubSys_Photonvision;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.DriveTrains.Cmds_SubSys_DriveTrain.Cmd_SubSys_DriveTrain_JoysticDefault;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;
import frc.robot.Library.Vision.Limelight.SubSys_LimeLight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public final SubSys_DriverStation driverStationSubSys = new SubSys_DriverStation();
    public Auto auto;
    /**
     * 
     * @param blingSubSys
     * @param photonvisionSubSys
     * @param handSubSys
     * @param armSubSys
     * @param gyroSubSys
     * @param driveSubSys
     */
    public RobotContainer(
            SubSys_Bling blingSubSys,
            SubSys_Photonvision photonvisionSubSys,
            SubSys_Hand handSubSys,
            SubSys_Arm armSubSys,
            SubSys_PigeonGyro gyroSubSys,
            SubSys_DriveTrain driveSubSys) {

        auto = new Auto(blingSubSys, photonvisionSubSys, handSubSys, armSubSys, gyroSubSys, driveSubSys);

        configureButtonBindings(blingSubSys, photonvisionSubSys, handSubSys, armSubSys, gyroSubSys, driveSubSys);

        // Configure default commands

        /** ***** Control System Components */
        armSubSys.setDefaultCommand(
                new Cmd_SubSys_Arm_JoysticDefault(
                        armSubSys,
                        () -> driverStationSubSys.GetArmRotateAxis(),
                        () -> driverStationSubSys.GetArmExtendAxis()));

        // handSubSys.setDefaultCommand(new Cmd_HandWithSensor(
        // handSubSys,
        // colorSubSys,
        // distanceSubsys,
        // () -> driverStationSubSys.HandSensorBtn())
        // );

        // mecanumDriveSubSys.setDefaultCommand(
        // new Cmd_MecanumDriveDefault(
        // mecanumDriveSubSys,
        // () -> driverStationSubSys.DriveFwdAxis(),
        // () -> driverStationSubSys.DriveStrAxis(),
        // () -> driverStationSubSys.DriveRotAxis()));

        driveSubSys.setDefaultCommand(
                new Cmd_SubSys_DriveTrain_JoysticDefault(
                        driveSubSys,
                        driverStationSubSys::DriveFwdAxis,
                        driverStationSubSys::DriveStrAxis,
                        driverStationSubSys::DriveRotAxis,
                        true,
                        driverStationSubSys::RotateLeftPt,
                        driverStationSubSys::RotateRightPt,
                        driverStationSubSys::DrivePerfModeAActive,
                        driverStationSubSys::DrivePerfModeBActive));
    }
    /**
     * 
     * @param armSubSys
     */
    public RobotContainer(SubSys_Arm armSubSys) {

        configureButtonBindingsSim(armSubSys);

        // Configure default commands

        /** ***** Control System Components */
        armSubSys.setDefaultCommand(
                new Cmd_SubSys_Arm_JoysticDefault(
                        armSubSys,
                        () -> driverStationSubSys.GetArmRotateAxis(),
                        () -> driverStationSubSys.GetArmExtendAxis()));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}. Use this method to
     * define your
     * button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one
     * of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     * 
     * @param driveSubSys
     * @param gyroSubSys
     * @param armSubSys
     * @param handSubSys
     * @param photonvisionSubSys
     * @param blingSubSys
     */
    private void configureButtonBindings(SubSys_Bling blingSubSys, SubSys_Photonvision photonvisionSubSys,
            SubSys_Hand handSubSys, SubSys_Arm armSubSys, SubSys_PigeonGyro gyroSubSys, SubSys_DriveTrain driveSubSys) {

        // Gyro Reset Command Button
        driverStationSubSys.OpenHandButton.onTrue(new InstantCommand(handSubSys::OpenHand, handSubSys));
        driverStationSubSys.CloseHandButton.onTrue(
                new InstantCommand(handSubSys::CloseHand, handSubSys));
        driverStationSubSys.GyroResetButton.onTrue(new InstantCommand(gyroSubSys::zeroYaw, gyroSubSys));

        // Gyro Reset Command Button
        driverStationSubSys.PoseResetButton.onTrue(
                // new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));
                new InstantCommand(driveSubSys::setPoseToOrigin, driveSubSys));

        // Test Button
        driverStationSubSys.TestButton.whileTrue(
                new Cmd_SubSys_Arm_PosCmd(armSubSys, -145.0, true, 1.54, true)
        // new CmdGrp_TestVisionAuto(driveSubSys, gyroSubSys, armSubSys, handSubSys,
        // blingSubSys,
        // photonvisionSubSys)
        );

        driverStationSubSys.GroundPickupButton.whileTrue(
                new Cmd_SubSys_Arm_PosCmd(armSubSys, 45.0, true, 0.8, true));

        driverStationSubSys.HighConeDelivery.whileTrue(
                new Cmd_SubSys_Arm_PosCmd(armSubSys, -35.0, true, 1.65, true));

        driverStationSubSys.MidConeDelivery.whileTrue(
                new Cmd_SubSys_Arm_PosCmd(armSubSys, -25.0, true, 1.00, true));

        driverStationSubSys.HighSafePos.whileTrue(
                new Cmd_SubSys_Arm_PosCmd(armSubSys, -80.0, true, 0.8, true));

        // CONE/CUBE SIGNALING
        driverStationSubSys.RequestConeButton.onTrue(
                new Cmd_SetBlingColorValue(
                        blingSubSys, Const_Bling.Controllers.controller1, Const_Bling.SolidColors.Yellow));
        driverStationSubSys.RequestCubeButton.onTrue(
                new Cmd_SetBlingColorValue(
                        blingSubSys, Const_Bling.Controllers.controller1, Const_Bling.SolidColors.Violet));

        // Fun signaling
        driverStationSubSys.ResetLEDColorButton.onTrue(
                new Cmd_SetBlingColorValue(
                        blingSubSys,
                        Const_Bling.Controllers.controller1,
                        Const_Bling.Patterns.Color1Color2.ColorWaves));
        driverStationSubSys.RainbowLEDColorButton.onTrue(
                new Cmd_SetBlingColorValue(
                        blingSubSys,
                        Const_Bling.Controllers.controller1,
                        Const_Bling.Patterns.FixedPalette.RainbowRainbow));
        driverStationSubSys.RainbowStrobeLEDColorButton.onTrue(
                new Cmd_SetBlingColorValue(
                        blingSubSys,
                        Const_Bling.Controllers.controller1,
                        Const_Bling.Patterns.FixedPalette.StrobeRed));
    }

    /**
     * Use this method to define your button->command mappings.
     * Only called during simulation
     *
     * @see configureButtonBindings
     * @param armSubSys
     */
    private void configureButtonBindingsSim(SubSys_Arm armSubSys) {

        driverStationSubSys.TestButton.whileTrue(
                new Cmd_SubSys_Arm_PosCmd(armSubSys, -145.0, true, 1.54, true)
        // new CmdGrp_TestVisionAuto(driveSubSys, gyroSubSys, armSubSys, handSubSys,
        // blingSubSys,
        // photonvisionSubSys)
        );

        driverStationSubSys.GroundPickupButton.whileTrue(
                new Cmd_SubSys_Arm_PosCmd(armSubSys, 45.0, true, 0.8, true));

        driverStationSubSys.HighConeDelivery.whileTrue(
                new Cmd_SubSys_Arm_PosCmd(armSubSys, -35.0, true, 1.65, true));

        driverStationSubSys.MidConeDelivery.whileTrue(
                new Cmd_SubSys_Arm_PosCmd(armSubSys, -25.0, true, 1.00, true));

        driverStationSubSys.HighSafePos.whileTrue(
                new Cmd_SubSys_Arm_PosCmd(armSubSys, -80.0, true, 0.8, true));
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
