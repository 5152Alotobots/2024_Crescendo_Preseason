package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.AutoCommands.Basic.*;
import frc.robot.ChargedUp.AutoCommands.SingleElement.Cube.*;
import frc.robot.ChargedUp.AutoCommands.TripleElement.*;
import frc.robot.ChargedUp.Bling.SubSys_Bling;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.ChargedUp.PhotonVision.SubSys_Photonvision;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;

public class Auto {
  private SubSys_Photonvision photonvisionSubSys;
  private SubSys_Bling blingSubSys;
  private SubSys_Hand handSubSys;
  private SubSys_Arm armSubSys;
  private SubSys_PigeonGyro gyroSubSys;
  private SubSys_DriveTrain driveSubSys;

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public Auto () {
    SubSys_DriveTrain driveSubSys = this.driveSubSys;
    SubSys_PigeonGyro gyroSubSys = this.gyroSubSys;
    SubSys_Arm armSubSys = this.armSubSys;
    SubSys_Hand handSubSys = this.handSubSys;
    SubSys_Bling blingSubSys = this.blingSubSys;
    SubSys_Photonvision photonvisionSubSys = this.photonvisionSubSys;

    Command m_stateescape = new Auto_stateescape_Cmd(driveSubSys, gyroSubSys);
    m_chooser.addOption("[BASIC] stateescape", m_stateescape);

    Command auto_Statemiddleleave_1cube_Cmd = new Auto_Statemiddleleave_1cube_Cmd(
            driveSubSys, gyroSubSys, armSubSys, handSubSys, blingSubSys);
    m_chooser.addOption("Statemiddleleave_1cube", auto_Statemiddleleave_1cube_Cmd);

    Command auto_Statebarrier_1cone1cube_red_Cmd = new Auto_Statebarrier_1cone1cube_red_Cmd(
            driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);
    m_chooser.addOption("StateBarrier - Red", auto_Statebarrier_1cone1cube_red_Cmd);

    Command auto_Statebarrier_1cone1cube_blue_Cmd = new Auto_Statebarrier_1cone1cube_blue_Cmd(
            driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);
    m_chooser.addOption("StateBarrier - Blue", auto_Statebarrier_1cone1cube_blue_Cmd);

    Command auto_autolink_red_Cmd = new Auto_autolink_red_Cmd(driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys);
    m_chooser.addOption("autolink - Red", auto_autolink_red_Cmd);

    Command auto_vision_blue_Cmd = new Auto_vision_blue_Cmd(
            driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys, photonvisionSubSys);
    m_chooser.addOption("vision - blue", auto_vision_blue_Cmd);

    Command auto_vision_red_Cmd = new Auto_vision_red_Cmd(
            driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys, photonvisionSubSys);
    m_chooser.addOption("vision - red", auto_vision_red_Cmd);

    Command auto_Statebestcharge_blue_Cmd = new Auto_Statebestcharge_blue_Cmd(
            driveSubSys, armSubSys, handSubSys, gyroSubSys, blingSubSys, photonvisionSubSys);
    m_chooser.addOption("Statebestcharge - blue", auto_Statebestcharge_blue_Cmd);

    Command auto_1cone2cubestashHPBlue_Cmd = new Auto_1cone2cubestashHPBlue_Cmd(
            driveSubSys, gyroSubSys, handSubSys, armSubSys, photonvisionSubSys, blingSubSys);
    m_chooser.addOption("[TRIPLE] 1 cone, 2 cube stash - Blue", auto_1cone2cubestashHPBlue_Cmd);

    Command auto_1cone2cubestashHPRed_Cmd = new Auto_1cone2cubestashHPRed_Cmd(
            driveSubSys, gyroSubSys, handSubSys, armSubSys, photonvisionSubSys, blingSubSys);
    m_chooser.addOption("[TRIPLE] 1 cone, 2 cube stash - Red", auto_1cone2cubestashHPRed_Cmd);

    Command auto_1cone2CubeHPBlue_Cmd = new Auto_1cone2cubeHPBlue_Cmd(
            driveSubSys, gyroSubSys, handSubSys, armSubSys, photonvisionSubSys, blingSubSys);
    m_chooser.addOption("[TRIPLE] 1 cone, 2 cube Human Player Side - Blue", auto_1cone2CubeHPBlue_Cmd);

    Command auto_1cone2CubeHPRed_Cmd = new Auto_1cone2cubeHPRed_Cmd(
            driveSubSys, gyroSubSys, handSubSys, armSubSys, photonvisionSubSys, blingSubSys);
    m_chooser.addOption("[TRIPLE] 1 cone, 2 cube Human Player Side - Red", auto_1cone2CubeHPRed_Cmd);

    SmartDashboard.putData(m_chooser);
  }

  public Command getAutoCommand() {
    return m_chooser.getSelected();
  }
}