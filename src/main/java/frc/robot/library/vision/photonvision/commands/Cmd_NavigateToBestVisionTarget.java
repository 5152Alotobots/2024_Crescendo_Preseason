/**
 * A command that navigates the robot to the best vision target using PhotonVision camera. The robot
 * is controlled based on the latest result obtained from the camera. The command ends when the
 * robot reaches the target.
 */
package frc.robot.library.vision.photonvision.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.bling.SubSys_Bling_Constants;
import frc.robot.subsystems.bling.SubSys_Bling;
import frc.robot.library.vision.photonvision.SubSys_Photonvision_Constants;
import frc.robot.library.vision.photonvision.SubSys_Photonvision_Constants.AcceptablePIDError;
import frc.robot.library.vision.photonvision.SubSys_Photonvision_Constants.PIDspeeds;
import frc.robot.library.vision.photonvision.SubSys_Photonvision;
import frc.robot.library.drivetrains.SubSys_DriveTrain;
import frc.robot.library.drivetrains.SubSys_DriveTrain_Constants.DriveTrainTrajSettings;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Cmd_NavigateToBestVisionTarget extends CommandBase {
  // Declare Variables
  private final SubSys_DriveTrain subSys_DriveTrain;
  private final SubSys_Photonvision subSys_Photonvision;
  private final SubSys_Bling subSys_Bling;
  private final int pipelineIndex;
  private final PhotonCamera camera;
  private final boolean enableX;
  private final boolean enableRotation;
  private final boolean enableY;
  private double acceptableDistanceToTarget;

  private PhotonPipelineResult latestResult;
  /* X PID */
  private PIDController Xcontroller =
      new PIDController(
          DriveTrainTrajSettings.DriveTrajectoryPID.Pgain * 3,
          DriveTrainTrajSettings.DriveTrajectoryPID.Igain,
          DriveTrainTrajSettings.DriveTrajectoryPID.Dgain);
  /* Y PID */
  private PIDController Ycontroller =
      new PIDController(
          DriveTrainTrajSettings.DriveTrajectoryPID.Pgain,
          DriveTrainTrajSettings.DriveTrajectoryPID.Igain,
          DriveTrainTrajSettings.DriveTrajectoryPID.Dgain);
  /* Z PID */
  private PIDController Zcontroller =
      new PIDController(
          DriveTrainTrajSettings.RotationTrajectoryPID.Pgain * 2.6,
          DriveTrainTrajSettings.RotationTrajectoryPID.Igain,
          DriveTrainTrajSettings.RotationTrajectoryPID.Dgain);

  /**
   * Constructor
   *
   * @param subSys_DriveTrain The subsystem used for controlling the robot's drivetrain
   * @param subSys_Photonvision The subsystem used for interfacing with the PhotonVision camera
   * @param camera The PhotonVision camera to use
   * @param pipelineIndex The index of the pipeline to use for targeting
   * @param enableX Whether to enable forward/backward movement
   * @param enableY Whether to enable strafing
   * @param enableRotation Whether to enable rotation
   *     <h4>NOTE: IF you choose to use enableY and enableRotation at the same time, unexpected
   *     results may occur
   */
  public Cmd_NavigateToBestVisionTarget(
      SubSys_DriveTrain subSys_DriveTrain,
      SubSys_Photonvision subSys_Photonvision,
      SubSys_Bling subSys_Bling,
      PhotonCamera camera,
      int pipelineIndex,
      boolean enableX,
      boolean enableY,
      boolean enableRotation) {
    this.subSys_DriveTrain = subSys_DriveTrain;
    this.subSys_Photonvision = subSys_Photonvision;
    this.subSys_Bling = subSys_Bling;
    this.pipelineIndex = pipelineIndex;
    this.camera = camera;
    this.enableRotation = enableRotation;
    this.enableX = enableX;
    this.enableY = enableY;
    addRequirements(subSys_DriveTrain, subSys_Photonvision, subSys_Bling);

    Xcontroller.setTolerance(0.01);
    Ycontroller.setTolerance(0.25);
    Zcontroller.setTolerance(0.3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.setPipelineIndex(pipelineIndex);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the distance where the robot should stop from a target
    switch (pipelineIndex) {
      case SubSys_Photonvision_Constants.Pipelines.Cube:
        acceptableDistanceToTarget = SubSys_Photonvision_Constants.TargetingConstants.Cube.GOAL_RANGE_METERS;
        break;
      case SubSys_Photonvision_Constants.Pipelines.Cone:
        acceptableDistanceToTarget = SubSys_Photonvision_Constants.TargetingConstants.Cone.GOAL_RANGE_METERS;
        break;
      case SubSys_Photonvision_Constants.Pipelines.Apriltag:
        acceptableDistanceToTarget =
            SubSys_Photonvision_Constants.TargetingConstants.GridApriltag.GOAL_RANGE_METERS;
        break;
    }
    // Get the latest result from the camera
    latestResult = camera.getLatestResult();
    SmartDashboard.putBoolean("At setpoint X VISION", Xcontroller.atSetpoint());
    SmartDashboard.putBoolean("At setpoint Y VISION", Ycontroller.atSetpoint());
    SmartDashboard.putBoolean("At setpoint Z VISION", Zcontroller.atSetpoint());
    SmartDashboard.putNumber("Error X PID VISION", Xcontroller.getPositionError());
    SmartDashboard.putNumber("Error Y PID VISION", Ycontroller.getPositionError());
    SmartDashboard.putNumber("Error Z PID VISION", Zcontroller.getPositionError());

    // Forward/Backward
    double xSpd;
    if (enableX) {
      xSpd = getVisionForwardSpeed(latestResult);
      SmartDashboard.putNumber("xSpd Vision", xSpd);
    } else {
      xSpd = 0;
    }
    // Strafe
    double ySpd;
    if (enableY) {
      ySpd = getVisionStrafeSpeed(latestResult);
      SmartDashboard.putNumber("ySpd Vision", ySpd);
    } else {
      ySpd = 0;
    }
    // Rotation
    double rotSpd;
    if (enableRotation) {
      rotSpd = getVisionRotSpeed(latestResult);
      SmartDashboard.putNumber("zSpd Vision", rotSpd);
    } else {
      rotSpd = 0;
    }

    subSys_DriveTrain.Drive(xSpd, ySpd, rotSpd, false, false, false);
    subSys_Bling.setBlinkinLEDColor(
        SubSys_Bling_Constants.Controllers.controller1, SubSys_Bling_Constants.Patterns.FixedPalette.BreathBlue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    subSys_DriveTrain.Drive(0, 0, 0, false, false, false);
    subSys_Bling.setBlinkinLEDColor(
        SubSys_Bling_Constants.Controllers.controller1, SubSys_Bling_Constants.SolidColors.Gold);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Boolean xFinished = true;
    Boolean yFinished = true;
    Boolean zFinished = true;
    // Check if the robot is at the target
    if (enableX) {
      xFinished =
          (Xcontroller.getPositionError() < AcceptablePIDError.X_PID_Error
              && Xcontroller.getPositionError() > -AcceptablePIDError.X_PID_Error);
    }
    if (enableY) {
      yFinished =
          (Ycontroller.getPositionError() < AcceptablePIDError.Y_PID_Error
              && Ycontroller.getPositionError() > -AcceptablePIDError.Y_PID_Error);
    }
    if (enableRotation) {
      zFinished =
          (Zcontroller.getPositionError() < AcceptablePIDError.Z_PID_Error
              && Zcontroller.getPositionError() > -AcceptablePIDError.Z_PID_Error);
    }
    return (xFinished && yFinished && zFinished);
  }

  /** Calculate forward speed */
  private double getVisionForwardSpeed(PhotonPipelineResult result) {
    if (result.hasTargets()) {
      return Math.min(
          Math.max(
              -Xcontroller.calculate(
                  subSys_Photonvision.getRangeToTarget(result, pipelineIndex),
                  acceptableDistanceToTarget),
              -PIDspeeds.Max_X_PID_Speed),
          PIDspeeds.Max_X_PID_Speed);
    } else {
      return 0;
    }
  }

  /** Calculate strafe speed */
  private double getVisionStrafeSpeed(PhotonPipelineResult result) {
    if (result.hasTargets()) {
      return Math.min(
          Math.max(
              Ycontroller.calculate((result.getBestTarget().getYaw()), 0),
              -PIDspeeds.Max_Y_PID_Speed),
          PIDspeeds.Max_Y_PID_Speed);
    } else {
      return 0;
    }
  }

  /** Calculate rotation speed */
  private double getVisionRotSpeed(PhotonPipelineResult result) {
    if (result.hasTargets()) {
      return Math.min(
          Math.max(
              Zcontroller.calculate(result.getBestTarget().getYaw(), 0),
              -PIDspeeds.Max_Z_PID_Speed),
          PIDspeeds.Max_Z_PID_Speed);
    } else {
      return 0;
    }
  }

  /** Returns true if the PID controllers don't need to move any further/you are at the target */
  private boolean isAtTarget(PhotonPipelineResult result) {
    if (result.hasTargets()) {
      return Xcontroller.atSetpoint() && Zcontroller.atSetpoint();
    } else {
      return false;
    }
  }
}
