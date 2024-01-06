// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library.drivetrains.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.library.drivetrains.SubSys_DriveTrain;
import frc.robot.library.drivetrains.SubSys_DriveTrain_Constants;

public class Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj extends CommandBase {
  /** Creates a new Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj. */
  private SubSys_DriveTrain subSys_DriveTrain;

  private String pathPlannerTrajName;
  private boolean setPose;
  private boolean setYaw;
  private Alliance setAlliance;
  private PathConstraints constraints;

  private PathPlannerTrajectory ppTraj;

  private final PIDController xDistancePID;
  private final PIDController yDistancePID;
  private final PIDController rotationPID;

  private Timer timer;

  /**
   * Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj Follow PathPlanner Trajectory
   *
   * @param subSys_DriveTrain SubSys_DriveTrain Drive Train subsystem
   * @param pathPlannerTraj String Name of PathPlannerTraj
   */
  public Cmd_SubSys_DriveTrain_FollowPathPlanner_Traj(
      SubSys_DriveTrain subSys_DriveTrain,
      String pathPlannerTrajName,
      boolean setPose,
      boolean setYaw,
      Alliance setAlliance) {

    this.subSys_DriveTrain = subSys_DriveTrain;
    this.pathPlannerTrajName = pathPlannerTrajName;
    this.setPose = setPose;
    this.setYaw = setYaw;
    this.setAlliance = setAlliance;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subSys_DriveTrain);

    // Load Path
    this.constraints = PathPlanner.getConstraintsFromPath(this.pathPlannerTrajName);
    this.ppTraj = PathPlanner.loadPath(this.pathPlannerTrajName, this.constraints);
    // new PathConstraints(2.7, 3.0));

    this.ppTraj = PathPlannerTrajectory.transformTrajectoryForAlliance(this.ppTraj, setAlliance);
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

    this.rotationPID =
        new PIDController(
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryPID.Pgain,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryPID.Igain,
            SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryPID.Dgain);
    this.rotationPID.enableContinuousInput(-180, 180);
    this.rotationPID.setTolerance(
        SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryPID.PositionTolerance,
        SubSys_DriveTrain_Constants.DriveTrainTrajSettings.RotationTrajectoryPID.VelocityTolerance);

    this.timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Check if the Odometry should be reset
    if (setYaw) {
      this.subSys_DriveTrain.setYaw(ppTraj.getInitialHolonomicPose().getRotation().getDegrees());
    }
    if (setPose) {
      this.subSys_DriveTrain.setPose(ppTraj.getInitialHolonomicPose());
    }

    this.xDistancePID.reset();
    this.xDistancePID.setSetpoint(ppTraj.getInitialHolonomicPose().getX());

    this.yDistancePID.reset();
    this.yDistancePID.setSetpoint(ppTraj.getInitialHolonomicPose().getY());

    this.rotationPID.reset();
    this.rotationPID.setSetpoint(ppTraj.getInitialHolonomicPose().getRotation().getDegrees());

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = this.timer.get();
    PathPlannerState desiredState = (PathPlannerState) ppTraj.sample(currentTime);

    Pose2d currentPose = subSys_DriveTrain.getPose();

    double xCmd = this.xDistancePID.calculate(currentPose.getX(), desiredState.poseMeters.getX());
    SmartDashboard.putNumber("xPID_xCmd", xCmd);

    double yCmd = this.yDistancePID.calculate(currentPose.getY(), desiredState.poseMeters.getY());
    SmartDashboard.putNumber("yPID_yCmd", yCmd);

    double rotCmd =
        this.rotationPID.calculate(
            this.subSys_DriveTrain.getHeading().getDegrees(),
            desiredState.holonomicRotation.getDegrees());
    SmartDashboard.putNumber("rotPID_rotCmd", rotCmd);

    this.subSys_DriveTrain.Drive(
        xCmd * SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveSpeedMultiplier,
        yCmd * SubSys_DriveTrain_Constants.DriveTrainTrajSettings.DriveSpeedMultiplier,
        rotCmd,
        true,
        false,
        false);

    SmartDashboard.putBoolean("x dist at set", this.xDistancePID.atSetpoint());
    SmartDashboard.putBoolean("y dist at set", this.yDistancePID.atSetpoint());
    SmartDashboard.putBoolean("rot dist at set", this.rotationPID.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.subSys_DriveTrain.Drive(0.0, 0.0, 0.0, false, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    PathPlannerState endState = ppTraj.getEndState();
    PathPlannerState currentState = (PathPlannerState) ppTraj.sample(this.timer.get());
    if (endState == currentState) {
      return true;
    }
    return false;
  }
}
