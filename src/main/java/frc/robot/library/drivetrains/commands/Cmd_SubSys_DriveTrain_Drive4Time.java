// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library.drivetrains.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.library.drivetrains.SubSys_DriveTrain;

public class Cmd_SubSys_DriveTrain_Drive4Time extends CommandBase {
  /** Creates a new Cmd_SubSys_DriveTrain_Drive4Time. */
  private final SubSys_DriveTrain subSys_DriveTrain;

  private final double driveFwdSpd;
  private final double driveStrSpd;
  private final double driveRotSpd;
  private final double driveTime;
  private final boolean fieldOriented;
  private final Timer timer = new Timer();

  public Cmd_SubSys_DriveTrain_Drive4Time(
      SubSys_DriveTrain subSys_DriveTrain,
      double driveFwdSpd,
      double driveStrSpd,
      double driveRotSpd,
      double driveTime,
      boolean fieldOriented) {

    this.subSys_DriveTrain = subSys_DriveTrain;
    this.driveFwdSpd = driveFwdSpd;
    this.driveStrSpd = driveStrSpd;
    this.driveRotSpd = driveRotSpd;
    this.driveTime = driveTime;
    this.fieldOriented = fieldOriented;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.subSys_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer.reset();
    this.timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.subSys_DriveTrain.Drive(
        this.driveFwdSpd, this.driveStrSpd, this.driveRotSpd, this.fieldOriented, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.subSys_DriveTrain.Drive(0.0, 0.0, 0.0, this.fieldOriented, false, false);
    this.timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (this.timer.get() >= this.driveTime) {
      return true;
    } else {
      return false;
    }
  }
}
