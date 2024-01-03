/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ChargedUp.Arm.SubSys_Arm;
import frc.robot.ChargedUp.Bling.SubSys_Bling;
import frc.robot.ChargedUp.Hand.SubSys_Hand;
import frc.robot.ChargedUp.PhotonVision.SubSys_Photonvision;
import frc.robot.Library.DriveTrains.SubSys_DriveTrain;
import frc.robot.Library.Gyroscopes.Pigeon2.SubSys_PigeonGyro;
import frc.robot.Library.Pneumatics.SubSys_Pneumatics;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command autonomousCommand;

  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

    if (RobotBase.isReal()) {
      final SubSys_Hand handSubSys = new SubSys_Hand();
      final SubSys_Arm armSubSys = new SubSys_Arm(handSubSys.getHandLength());
      final SubSys_Bling blingSubSys = new SubSys_Bling();
      final SubSys_PigeonGyro gyroSubSys = new SubSys_PigeonGyro();
      final SubSys_DriveTrain driveSubSys = new SubSys_DriveTrain(gyroSubSys);
      final SubSys_Photonvision photonSubSys = new SubSys_Photonvision();
      final SubSys_Pneumatics pneumaticsSubSys = new SubSys_Pneumatics();
      
      robotContainer = new RobotContainer(blingSubSys, photonSubSys, handSubSys, armSubSys, gyroSubSys, driveSubSys);
      CameraServer.startAutomaticCapture();
      pneumaticsSubSys.compressorOn();
    } else {
      final SubSys_Hand handSubSys = new SubSys_Hand();
      final SubSys_Arm armSubSys = new SubSys_Arm(handSubSys.getHandLength());
      robotContainer = new RobotContainer(armSubSys);
    }

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    if (RobotBase.isReal()) {
      autonomousCommand = robotContainer.getAutonomousCommand();

      // schedule the autonomous command (example)
      if (autonomousCommand != null) {
        autonomousCommand.schedule();
      }
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // m_robotContainer.m_DriveSubSys.resetDriveMotorEncoders();
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
