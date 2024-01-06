// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubSys_Pneumatics extends SubsystemBase {
  /** Creates a new SubSys_Pneumatics. */
  public SubSys_Pneumatics() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  Compressor pcmCompressor = new Compressor(21, PneumaticsModuleType.CTREPCM);

  public void compressorOn() {
    pcmCompressor.enableDigital();
  }
}
