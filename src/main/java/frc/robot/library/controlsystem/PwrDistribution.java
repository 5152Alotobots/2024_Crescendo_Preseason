/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.library.controlsystem;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;

public class PwrDistribution extends SubsystemBase {
  /** Creates a new Power distribution Subsystem. */
  PowerDistribution pdp = new PowerDistribution(CAN_IDs.PDP_CAN_ID, ModuleType.kRev);

  public PwrDistribution() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
