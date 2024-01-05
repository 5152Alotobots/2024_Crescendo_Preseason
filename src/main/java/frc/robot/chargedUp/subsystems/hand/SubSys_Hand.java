// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup.subsystems.hand;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Robot;

public class SubSys_Hand extends SubsystemBase {
  /** Creates a new SubSys_Hand. */
  public final DoubleSolenoid HandSolenoid =
      new DoubleSolenoid(21, PneumaticsModuleType.CTREPCM, 0, 1);

  public SubSys_Hand() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void OpenHand() {
    HandSolenoid.set(Value.kReverse);
  }

  public void CloseHand() {
    HandSolenoid.set(Value.kForward);
  }

  public Value getHandState() {
    return HandSolenoid.get();
  }

  public double getHandLength() {
    double handLength = Robot.Dimensions.Hand.HandForwardLength;
    if (HandSolenoid.get() == Value.kReverse) {
      handLength = Robot.Dimensions.Hand.HandRetractLength;
    }
    return handLength;
  }
}
