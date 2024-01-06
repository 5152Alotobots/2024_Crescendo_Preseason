// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class DriverStation {

  // Driver Controller
  private final XboxController m_DriverController = new XboxController(0);

  public JoystickButton GyroResetButton = new JoystickButton(m_DriverController, 4);
  public JoystickButton CloseHandButton = new JoystickButton(m_DriverController, 6);
  public JoystickButton OpenHandButton = new JoystickButton(m_DriverController, 5);
  public JoystickButton PoseResetButton = new JoystickButton(m_DriverController, 1);
  public JoystickButton TestButton = new JoystickButton(m_DriverController, 3);

  // Co-Driver Controller
  private final XboxController m_CoDriverController = new XboxController(1);
  public JoystickButton GroundPickupButton = new JoystickButton(m_CoDriverController, 1);
  public JoystickButton HighConeDelivery = new JoystickButton(m_CoDriverController, 4);
  public JoystickButton MidConeDelivery = new JoystickButton(m_CoDriverController, 3);
  public JoystickButton HighSafePos = new JoystickButton(m_CoDriverController, 2);
  public JoystickButton RequestConeButton = new JoystickButton(m_CoDriverController, 6);
  public JoystickButton RequestCubeButton = new JoystickButton(m_CoDriverController, 5);
  public POVButton ResetLEDColorButton = new POVButton(m_CoDriverController, 180);
  public POVButton RainbowLEDColorButton = new POVButton(m_CoDriverController, 0);
  public POVButton RainbowStrobeLEDColorButton = new POVButton(m_CoDriverController, 270);

  // AuxDriver Controller
  //private final XboxController m_AuxDriverController = new XboxController(2);

  public DriverStation() {}

  // ---- Drive Subsystem
  // Drive Fwd Axis
  public double DriveFwdAxis() {
    return -m_DriverController.getRawAxis(1);
  }

  // Drive Strafe Axis
  public double DriveStrAxis() {
    return -m_DriverController.getRawAxis(0);
  }

  // Drive Rotate Axis
  public double DriveRotAxis() {
    return -m_DriverController.getRawAxis(4);
  }

  // Drive RotateLeftPoint
  public boolean RotateLeftPt() {
    return m_DriverController.getRawButton(5);
  }

  // Drive RotateRightPoint
  public boolean RotateRightPt() {
    return m_DriverController.getRawButton(6);
  }

  // Drive Performance Mode A
  public boolean DrivePerfModeAActive() {
    return (m_DriverController.getRawAxis(2) > 0.3);
  }

  // Drive Performance Mode B
  public boolean DrivePerfModeBActive() {
    return (m_DriverController.getRawAxis(3) > 0.3);
  }

  // ----- Arm Subsystem
  // Arm ShoulderRotate Axis
  public double GetArmRotateAxis() {
    return m_CoDriverController.getRawAxis(1);
  }

  // Arm Extend Axis
  public double GetArmExtendAxis() {
    return -m_CoDriverController.getRawAxis(5);
  }

}
