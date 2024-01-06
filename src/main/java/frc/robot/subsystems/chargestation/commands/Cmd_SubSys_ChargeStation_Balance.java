// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chargestation.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.library.drivetrains.SubSys_DriveTrain;
import frc.robot.library.gyroscopes.pigeon2.SubSys_PigeonGyro;
import frc.robot.subsystems.chargestation.SubSys_ChargeStation_Constants;

public class Cmd_SubSys_ChargeStation_Balance extends CommandBase {
  /** Creates a new Cmd_AutoBalance. */
  private final SubSys_PigeonGyro m_PigeonGyro;

  private final SubSys_DriveTrain m_DriveTrain;
  Timer m_endTimer_seconds = new Timer();
  Boolean newMovement = true;
  Boolean finished = false;

  public Cmd_SubSys_ChargeStation_Balance(SubSys_PigeonGyro pigeonGyro, SubSys_DriveTrain driveTrain) {
    m_PigeonGyro = pigeonGyro;
    m_DriveTrain = driveTrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_PigeonGyro, m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endTimer_seconds.reset();
    m_endTimer_seconds.start();
    newMovement = true;
    finished = false;
    speed = 0.040;
  }

  public double speed;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * This code will check the angle that the robot is at and drive in the direction that is needed
     * to balance the robot
     */
    double pitch = m_PigeonGyro.getRawGyroPitch();
    double roll = m_PigeonGyro.getRawGyroRoll();

    if (newMovement) {

      if (pitch > SubSys_ChargeStation_Constants.maxPitch
          || pitch < SubSys_ChargeStation_Constants.minPitch) {
        // Drive using the pitch as the speed. (gyro angle * 0.01 if the gyro is 8
        // degrees off, the robot will drive at 0.08 speed)
        m_DriveTrain.Drive(
            (pitch * speed) * SubSys_ChargeStation_Constants.speedMultiplier,
            0,
            0,
            false,
            false,
            false);
      } else if (roll > SubSys_ChargeStation_Constants.maxRoll
          || roll < SubSys_ChargeStation_Constants.minRoll) {
        // Drive using the roll as the speed.
        m_DriveTrain.Drive(
            0,
            (roll * speed) * SubSys_ChargeStation_Constants.speedMultiplier,
            0,
            false,
            false,
            false);
      } else {
        finished = true;
      }
      newMovement = false;
      if (speed > 0.015) {
        speed = speed - 0.0025;
      }

    } else if (m_endTimer_seconds.get() >= 2.25) {
      if (pitch > SubSys_ChargeStation_Constants.maxPitch
          || pitch < SubSys_ChargeStation_Constants.minPitch
          || roll > SubSys_ChargeStation_Constants.maxRoll
          || roll < SubSys_ChargeStation_Constants.minRoll) {
        newMovement = true;
        m_endTimer_seconds.reset();
        m_endTimer_seconds.start();

      } else {
        finished = true;
      }
      // changed to 0.5 from 1
    } else if (m_endTimer_seconds.get() >= 0.50) {
      m_DriveTrain.Drive(0, 0, 0, false, false, false);
    }

    /*  OLD CODe
        if (m_endTimer_seconds.get() < SubSys_ChargeStation_Constants.autoBalanceIntervalSeconds) {


          if (pitch > SubSys_ChargeStation_Constants.maxPitch || pitch < SubSys_ChargeStation_Constants.minPitch) {
            // Drive using the pitch as the speed. (gyro angle * 0.01 if the gyro is 8
            // degrees off, the robot will drive at 0.08 speed)
            m_DriveTrain.Drive(-pitch * 0.01, 0, 0, false, false, false);
          } else if (roll > SubSys_ChargeStation_Constants.maxRoll || roll < SubSys_ChargeStation_Constants.minRoll) {
            // Drive using the roll as the speed.
            m_DriveTrain.Drive(0, roll * 0.01, 0, false, false, false);
          }
        }
      }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (finished) {
      return true;
    } else {
      return false;
    }
  }
}
