// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library.vision.photonvision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class SubSys_Photonvision extends SubsystemBase {

  /** Creates a new PhotonVisionSubsytem. */
  public SubSys_Photonvision() {

    /* CONFIG PID */
    // Log target data to the dashboard (shuffleboard)
    // Shuffleboard.getTab("PhotonVision").add("Pipeline Index", pipelineIndex);
    // Shuffleboard.getTab("PhotonVision").add("Range to Target", rangeToTarget);
    // Shuffleboard.getTab("PhotonVision").add("Target Area", targetArea);
    // Shuffleboard.getTab("PhotonVision").add("Target Pitch", targetPitch);
    // Shuffleboard.getTab("PhotonVision").add("Target Yaw", targetYaw);
    // Shuffleboard.getTab("PhotonVision").add("Target Skew", targetSkew);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Log target data to the dashboard if we have a target
    PhotonPipelineResult result = SubSys_Photonvision_Constants.Cameras.frontCamera.getLatestResult();
    if (result.hasTargets()) {
      int pipelineIndex = SubSys_Photonvision_Constants.Cameras.frontCamera.getPipelineIndex();
      double rangeToTarget = getRangeToTarget(result, pipelineIndex);
      double targetArea = result.getBestTarget().getArea();
      double targetPitch = result.getBestTarget().getPitch();
      double targetYaw = result.getBestTarget().getYaw();
      double targetSkew = result.getBestTarget().getSkew();

      // Log target data to the dashboard (Shuffleboard)
      // Shuffleboard.update();

      // Log target data to the dashboard (SmartDashboard)
      SmartDashboard.putNumber("Pipeline Index - Front cam", pipelineIndex);
      SmartDashboard.putNumber("Range to Target", rangeToTarget);
      SmartDashboard.putNumber("Target Area", targetArea);
      SmartDashboard.putNumber("Target Pitch", targetPitch);
      SmartDashboard.putNumber("Target Yaw", targetYaw);
      SmartDashboard.putNumber("Target Skew", targetSkew);
    }
  }

  /* VISION */
  /** Calculate distance to target */
  public double getRangeToTarget(PhotonPipelineResult result, int pipelineIndex) {
    double TARGET_HEIGHT_METERS;
    if (result.hasTargets()) {
      switch (pipelineIndex) {
        default:
          TARGET_HEIGHT_METERS = SubSys_Photonvision_Constants.TargetingConstants.Cube.TARGET_HEIGHT_METERS;
          break;
        case SubSys_Photonvision_Constants.Pipelines.Cone:
          TARGET_HEIGHT_METERS = SubSys_Photonvision_Constants.TargetingConstants.Cone.TARGET_HEIGHT_METERS;
          break;
        case SubSys_Photonvision_Constants.Pipelines.Apriltag:
          TARGET_HEIGHT_METERS =
              SubSys_Photonvision_Constants.TargetingConstants.GridApriltag.TARGET_HEIGHT_METERS;
          break;
      }
      return PhotonUtils.calculateDistanceToTargetMeters(
          SubSys_Photonvision_Constants.CAMERA_HEIGHT_METERS,
          TARGET_HEIGHT_METERS,
          SubSys_Photonvision_Constants.CAMERA_PITCH_RADIANS,
          Units.degreesToRadians(result.getBestTarget().getPitch()));
    } else {
      return 0;
    }
  }

  /**
   * Returns true if the camera's target is taking up the acceptable percentage of the viewport
   *
   * @param acceptablePercentage The percentage of the viewport the target should take up 0-1
   * @param result The result of the pipeline
   * @return True if the target is taking up the acceptable percentage of the viewport
   */
  public boolean isInRange(PhotonPipelineResult result, double acceptablePercentage) {
    if (result.hasTargets()) {
      return result.getBestTarget().getArea() >= acceptablePercentage;
    } else {
      return false;
    }
  }
}
