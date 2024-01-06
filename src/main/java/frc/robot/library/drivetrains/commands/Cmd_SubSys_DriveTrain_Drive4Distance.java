// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library.drivetrains.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.library.drivetrains.SubSys_DriveTrain;
import frc.robot.library.drivetrains.SubSys_DriveTrain_Constants;

public class Cmd_SubSys_DriveTrain_Drive4Distance extends CommandBase {
  /** Creates a new Cmd_SubSyst_DriveTrain_Drive4Distance. */
  private final SubSys_DriveTrain subSys_DriveTrain;

  private double targetXDistance;
  private double initialXDistance;
  private final PIDController xDistancePID;
  // private final ProfiledPIDController xDistancePID;
  private double targetYDistance;
  private double initialYDistance;
  private final PIDController yDistancePID;
  // private final ProfiledPIDController yDistancePID;
  private double targetHeadingDegrees;
  private final ProfiledPIDController profiledRotationPID;

  private final TrapezoidProfile.Constraints profiledDriveConstraints;
  private final TrapezoidProfile.Constraints profiledRotationConstraints;
  private final SimpleMotorFeedforward feedForward;

  public Cmd_SubSys_DriveTrain_Drive4Distance(
      SubSys_DriveTrain subSys_DriveTrain,
      double targetXDistance,
      double targetYDistance,
      double targetHeadingDegrees) {

    this.subSys_DriveTrain = subSys_DriveTrain;
    this.targetXDistance = targetXDistance;
    this.initialXDistance = 0.0;
    this.targetYDistance = targetYDistance;
    this.initialYDistance = 0.0;
    this.targetHeadingDegrees = targetHeadingDegrees;

    this.feedForward =
        new SimpleMotorFeedforward(
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryFF.kS,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryFF.kV);

    this.profiledDriveConstraints =
        new TrapezoidProfile.Constraints(
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrainTrajMaxSpd,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrainTrajMaxAccel);

    this.xDistancePID =
        new PIDController(
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Pgain,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Igain,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Dgain);
    this.xDistancePID.setTolerance(
        SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.PositionTolerance,
        SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.VelocityTolerance);

    this.yDistancePID =
        new PIDController(
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Pgain,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Igain,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Dgain);
    this.yDistancePID.setTolerance(
        SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.PositionTolerance,
        SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.VelocityTolerance);

    /*
    this.xDistancePID = new ProfiledPIDController(
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Pgain,
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Igain,
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Dgain,
      this.profiledMoveConstraints);

    this.yDistancePID = new ProfiledPIDController(
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Pgain,
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Igain,
      SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrajectoryPID.Dgain,
      this.profiledMoveConstraints);
    */
    this.profiledRotationConstraints =
        new TrapezoidProfile.Constraints(
            Units.radiansToDegrees(
                SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrainTrajMaxRotSpeed),
            Units.radiansToDegrees(
                SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveTrainTrajMaxRotAccel));

    this.profiledRotationPID =
        new ProfiledPIDController(
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryPID.Pgain,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryPID.Igain,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryPID.Dgain,
            this.profiledRotationConstraints);

    this.profiledRotationPID.enableContinuousInput(-180, 180);
    this.profiledRotationPID.setTolerance(
        SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryPID.PositionTolerance,
        SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryPID.VelocityTolerance);
    this.profiledRotationPID.setIntegratorRange(-.3, 0.3);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.subSys_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.subSys_DriveTrain.setPose(new Pose2d());
    Pose2d currPose = this.subSys_DriveTrain.getPose();
    this.initialXDistance = currPose.getX();
    this.xDistancePID.reset();
    this.xDistancePID.setSetpoint(this.targetXDistance);

    this.initialYDistance = currPose.getY();
    this.yDistancePID.reset();
    this.yDistancePID.setSetpoint(this.targetYDistance);

    this.profiledRotationPID.reset(this.subSys_DriveTrain.getHeading().getDegrees());
    this.profiledRotationPID.setGoal(this.targetHeadingDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currPose = this.subSys_DriveTrain.getPose();

    double xCmd =
        this.xDistancePID.calculate(currPose.getX() - this.initialXDistance, this.targetXDistance);

    SmartDashboard.putNumber("xPID", xCmd);

    double yCmd =
        this.yDistancePID.calculate(currPose.getY() - this.initialYDistance, this.targetYDistance);

    SmartDashboard.putNumber("yPID", yCmd);

    double rotFFCmd = this.feedForward.calculate(this.profiledRotationPID.getSetpoint().velocity);
    double rotPIDCmd =
        this.profiledRotationPID.calculate(
            this.subSys_DriveTrain.getHeading().getDegrees(), this.targetHeadingDegrees);

    double rotCmd = rotFFCmd + rotPIDCmd;

    SmartDashboard.putNumber("rotPID", rotCmd);
    this.subSys_DriveTrain.Drive(xCmd, yCmd, rotCmd, true, false, false);

    SmartDashboard.putNumber("Drive4Distance_xSetpoint", this.xDistancePID.getSetpoint());
    SmartDashboard.putNumber("Drive4Distance_xError", this.xDistancePID.getPositionError());
    SmartDashboard.putNumber("Drive4Distance_xCmd", xCmd);
    SmartDashboard.putBoolean("Drive4Distance_AtSetpoint", this.xDistancePID.atSetpoint());
    // SmartDashboard.putNumber("Rotate2Heading_Setpoint_Position",
    // this.profiledRotationPID.getSetpoint().position);
    // SmartDashboard.putNumber("Rotate2Heading_SetPoint_Velocity",this.profiledRotationPID.getSetpoint().velocity);
    // SmartDashboard.putNumber("Rotate2Heading_Error",
    // this.profiledRotationPID.getPositionError());

    // SmartDashboard.putNumber("Rotate2Heading_rotPIDCmd", rotPIDCmd);
    // SmartDashboard.putNumber("Rotate2Heading_rotFFCmd", rotFFCmd);
    // SmartDashboard.putNumber("Rotate2Heading_rotCmd", rotCmd);

    /* SETPOINTS */
    // X
    // SmartDashboard.putNumber("Drive4Dist_Setpoint_Position X",
    // this.xDistancePID.getSetpoint().position);
    // SmartDashboard.putNumber("Drive4Dist_SetPoint_Velocity
    // X",this.xDistancePID.getSetpoint().velocity);
    // Y
    // SmartDashboard.putNumber("Drive4Dist_Setpoint_Position Y",
    // this.yDistancePID.getSetpoint().position);
    // SmartDashboard.putNumber("Drive4Dist_SetPoint_Velocity
    // Y",this.yDistancePID.getSetpoint().velocity);
    // ROT
    SmartDashboard.putNumber(
        "Drive4Dist_Setpoint_Position ROT", this.profiledRotationPID.getSetpoint().position);
    SmartDashboard.putNumber(
        "Drive4Dist_SetPoint_Velocity ROT", this.profiledRotationPID.getSetpoint().velocity);

    /* GOALS */
    // X
    // SmartDashboard.putNumber("Drive4Dist_Goal_Position X", this.xDistancePID.getGoal().position);
    // SmartDashboard.putNumber("Drive4Dist_Goal_Velocity X", this.xDistancePID.getGoal().velocity);
    // Y
    // SmartDashboard.putNumber("Drive4Dist_Goal_Position Y", this.yDistancePID.getGoal().position);
    // SmartDashboard.putNumber("Drive4Dist_Goal_Velocity Y", this.yDistancePID.getGoal().velocity);
    // ROT
    SmartDashboard.putNumber(
        "Drive4Dist_Goal_Position ROT", this.profiledRotationPID.getGoal().position);
    SmartDashboard.putNumber(
        "Drive4Dist_Goal_Velocity ROT", this.profiledRotationPID.getGoal().velocity);

    /* ERRORS */
    // X
    SmartDashboard.putNumber("Drive4Dist_Velocity Error X", this.xDistancePID.getVelocityError());
    SmartDashboard.putNumber("Drive4Dist_Error X", this.xDistancePID.getPositionError());
    // Y
    SmartDashboard.putNumber("Drive4Dist_Velocity Error Y", this.yDistancePID.getVelocityError());
    SmartDashboard.putNumber("Drive4Dist_Error Y", this.yDistancePID.getPositionError());
    // ROT
    SmartDashboard.putNumber(
        "Drive4Dist_Velocity Error ROT", this.profiledRotationPID.getVelocityError());
    SmartDashboard.putNumber("Drive4Dist_Error ROT", this.profiledRotationPID.getPositionError());

    /* PIDF GAINS */
    // X
    SmartDashboard.putNumber("Drive4Dist_Pgain X", this.xDistancePID.getP());
    SmartDashboard.putNumber("Drive4Dist_Igain X", this.xDistancePID.getI());
    SmartDashboard.putNumber("Drive4Dist_Dgain X", this.xDistancePID.getD());
    // Y
    SmartDashboard.putNumber("Drive4Dist_Pgain Y", this.yDistancePID.getP());
    SmartDashboard.putNumber("Drive4Dist_Igain Y", this.yDistancePID.getI());
    SmartDashboard.putNumber("Drive4Dist_Dgain Y", this.yDistancePID.getD());
    // ROT
    SmartDashboard.putNumber("Drive4Dist_Pgain ROT", this.profiledRotationPID.getP());
    SmartDashboard.putNumber("Drive4Dist_Igain ROT", this.profiledRotationPID.getI());
    SmartDashboard.putNumber("Drive4Dist_Dgain ROT", this.profiledRotationPID.getD());

    /* Is At Goal */
    // X
    // SmartDashboard.putBoolean("Drive4Dist_IsAtGoal X", this.xDistancePID.atGoal());
    // Y
    // SmartDashboard.putBoolean("Drive4Dist_IsAtGoal Y", this.yDistancePID.atGoal());
    // ROT
    SmartDashboard.putBoolean("Drive4Dist_IsAtGoal ROT", this.profiledRotationPID.atGoal());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.subSys_DriveTrain.Drive(0.0, 0.0, 0.0, false, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* PID */
    if (this.xDistancePID.atSetpoint()
        && this.yDistancePID.atSetpoint()
        && this.profiledRotationPID.atGoal()) {
      return true;
    } else {
      return false;
    }
    /* ProfiledPID
    if (this.xDistancePID.atGoal() &&
         this.yDistancePID.atGoal() &&
         this.profiledRotationPID.atGoal()){
      return true;
    }else{
      return false;
    }
    */
  }
}
