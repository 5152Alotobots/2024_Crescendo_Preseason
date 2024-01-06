package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.SubSys_Arm;
import frc.robot.commands.auto.basic.*;
import frc.robot.commands.auto.singleelement.cube.*;
import frc.robot.commands.auto.tripleelement.*;
import frc.robot.subsystems.bling.SubSys_Bling;
import frc.robot.subsystems.hand.SubSys_Hand;
import frc.robot.library.vision.photonvision.SubSys_Photonvision;
import frc.robot.library.drivetrains.SubSys_DriveTrain;
import frc.robot.library.gyroscopes.pigeon2.SubSys_PigeonGyro;

/**
 * The Auto class is responsible for handling the autonomous mode of the robot. It includes a
 * variety of commands for different tasks such as escaping, state middle leave, state barrier, and
 * others. The class also includes a SendableChooser object to select the desired command.
 */
public class Auto {
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** Constructor for the Auto class. Initializes all the subsystems and commands. */
  public Auto(SubSys_Bling blingSubSys, SubSys_Photonvision photonvisionSubSys, SubSys_Hand handSubSys, SubSys_Arm armSubSys, SubSys_PigeonGyro gyroSubSys, SubSys_DriveTrain driveSubSys) {


    Command m_stateescape = new Auto_stateescape_Cmd(driveSubSys, gyroSubSys);
    m_chooser.addOption("[BASIC] stateescape", m_stateescape);

    Command auto_Statemiddleleave_1cube_Cmd =
        new Auto_Statemiddleleave_1cube_Cmd(
            driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys);
    m_chooser.addOption("Statemiddleleave_1cube", auto_Statemiddleleave_1cube_Cmd);

    Command auto_Statebarrier_1cone1cube_red_Cmd =
        new Auto_Statebarrier_1cone1cube_red_Cmd(
            driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);
    m_chooser.addOption("StateBarrier - Red", auto_Statebarrier_1cone1cube_red_Cmd);

    Command auto_Statebarrier_1cone1cube_blue_Cmd =
        new Auto_Statebarrier_1cone1cube_blue_Cmd(
            driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);
    m_chooser.addOption("StateBarrier - Blue", auto_Statebarrier_1cone1cube_blue_Cmd);

    Command auto_autolink_red_Cmd =
        new Auto_autolink_red_Cmd(driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);
    m_chooser.addOption("autolink - Red", auto_autolink_red_Cmd);

    Command auto_vision_blue_Cmd =
        new Auto_vision_blue_Cmd(
            driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys, photonvisionSubSys);
    m_chooser.addOption("vision - blue", auto_vision_blue_Cmd);

    Command auto_vision_red_Cmd =
        new Auto_vision_red_Cmd(
            driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys, photonvisionSubSys);
    m_chooser.addOption("vision - red", auto_vision_red_Cmd);

    Command auto_Statebestcharge_blue_Cmd =
        new Auto_Statebestcharge_blue_Cmd(
            driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys, photonvisionSubSys);
    m_chooser.addOption("Statebestcharge - blue", auto_Statebestcharge_blue_Cmd);

    Command auto_1cone2cubestashHPBlue_Cmd =
        new Auto_1cone2cubestashHPBlue_Cmd(
            driveSubSys, gyroSubSys, handSubSys, armSubSys, photonvisionSubSys, blingSubSys);
    m_chooser.addOption("[TRIPLE] 1 cone, 2 cube stash - Blue", auto_1cone2cubestashHPBlue_Cmd);

    Command auto_1cone2cubestashHPRed_Cmd =
        new Auto_1cone2cubestashHPRed_Cmd(
            driveSubSys, gyroSubSys, handSubSys, armSubSys, photonvisionSubSys, blingSubSys);
    m_chooser.addOption("[TRIPLE] 1 cone, 2 cube stash - Red", auto_1cone2cubestashHPRed_Cmd);

    Command auto_1cone2CubeHPBlue_Cmd =
        new Auto_1cone2cubeHPBlue_Cmd(
            driveSubSys, gyroSubSys, handSubSys, armSubSys, photonvisionSubSys, blingSubSys);
    m_chooser.addOption(
        "[TRIPLE] 1 cone, 2 cube Human Player Side - Blue", auto_1cone2CubeHPBlue_Cmd);

    Command auto_1cone2CubeHPRed_Cmd =
        new Auto_1cone2cubeHPRed_Cmd(
            driveSubSys, gyroSubSys, handSubSys, armSubSys, photonvisionSubSys, blingSubSys);
    m_chooser.addOption(
        "[TRIPLE] 1 cone, 2 cube Human Player Side - Red", auto_1cone2CubeHPRed_Cmd);


    SmartDashboard.putData(m_chooser);
  }

  /**
   * Get the selected command from the SendableChooser.
   *
   * @return The selected command.
   */
  public Command getAutoCommand() {
    return m_chooser.getSelected();
  }
}
