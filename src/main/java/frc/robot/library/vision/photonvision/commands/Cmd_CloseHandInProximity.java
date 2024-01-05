/**
 * A command that closes the hand when a target is detected within range using a PhotonVision
 * camera.
 *
 * <p>This command uses a PhotonVision camera and a specified pipeline to detect a target. When the
 * target is detected within range, the hand is closed.
 */
package frc.robot.library.vision.photonvision.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.chargedup.subsystems.hand.SubSys_Hand;
import frc.robot.library.vision.photonvision.SubSys_Photonvision;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Cmd_CloseHandInProximity extends CommandBase {

  private final SubSys_Photonvision subSys_Photonvision;
  private final SubSys_Hand subSys_Hand;
  private final int pipelineIndex;
  private final PhotonCamera camera;
  private PhotonPipelineResult result;
  private Boolean isFinished = false;

  /**
   * Constructs a new {@link Cmd_CloseHandInProximity} command with the specified PhotonVision
   * camera, pipeline index, and hand subsystem.
   *
   * @param subSys_Photonvision the PhotonVision subsystem
   * @param subSys_Hand the hand subsystem
   * @param camera the PhotonVision camera
   * @param pipelineIndex the index of the pipeline to use
   */
  public Cmd_CloseHandInProximity(
      SubSys_Photonvision subSys_Photonvision,
      SubSys_Hand subSys_Hand,
      PhotonCamera camera,
      int pipelineIndex) {
    this.subSys_Photonvision = subSys_Photonvision;
    this.subSys_Hand = subSys_Hand;
    this.pipelineIndex = pipelineIndex;
    this.camera = camera;
    addRequirements(subSys_Photonvision, subSys_Hand);
  }

  /** Sets the pipeline index of the camera when the command is initially scheduled. */
  @Override
  public void initialize() {
    camera.setPipelineIndex(pipelineIndex);
  }

  /** Gets the latest result from the camera and closes the hand if the target is within range. */
  @Override
  public void execute() {
    result = camera.getLatestResult();

    if (subSys_Photonvision.isInRange(result, 0.6)) {
      subSys_Hand.CloseHand();
      isFinished = true;
    }
  }

  /**
   * Called when the command ends or is interrupted. Does nothing in this implementation.
   *
   * @param interrupted whether the command was interrupted
   */
  @Override
  public void end(boolean interrupted) {}

  /**
   * Returns whether the command has finished.
   *
   * @return whether the command has finished
   */
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
