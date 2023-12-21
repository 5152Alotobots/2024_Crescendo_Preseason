// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChargedUp.Arm;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Robot;
import frc.robot.Library.MotorControllers.TalonFX.TalonFX_Conversions;
import edu.wpi.first.wpilibj.RobotController;
public class SubSys_Arm extends SubsystemBase {

  private final TalonFX Arm_ShoulderMotor =
          new TalonFX(Constants.CAN_IDs.ARM_SHOULDER_MTR_CAN_ID);
  private final TalonFXSimCollection Arm_ShoulderMotorSim = Arm_ShoulderMotor.getSimCollection();

  private final TalonFX Arm_ShoulderFollowerMotor =
          new TalonFX(Constants.CAN_IDs.ARM_SHOULDER_FOLLOWER_MTR_CAN_ID);
  private final TalonFXSimCollection Arm_ShoulderFollowerMotorSim = Arm_ShoulderFollowerMotor.getSimCollection();

  private final TalonFX Arm_ExtensionMotor =
          new TalonFX(Constants.CAN_IDs.ARM_EXTENSION_MTR_CAN_ID);
  private final TalonFXSimCollection Arm_ExtensionMotorSim = Arm_ExtensionMotor.getSimCollection();


  private final CANCoder Arm_ShoulderEncoder =
          new CANCoder(Constants.CAN_IDs.ARM_SHOULDER_CAN_CODER_CAN_ID);
  private final CANCoderSimCollection Arm_ShoulderEncoderSim = Arm_ShoulderEncoder.getSimCollection();

  private final CANCoder Arm_ExtensionEncoder =
          new CANCoder(Constants.CAN_IDs.ARM_EXTENSION_CAN_CODER_CAN_ID);
  private final CANCoderSimCollection Arm_ExtensionEncoderSim = Arm_ExtensionEncoder.getSimCollection();

  /**
   * Arm Stop Switch reports false when fully retracted and true when not
   */
  private final DigitalInput Arm_Extension_RetractStopSwitch = new DigitalInput(0);
  private DigitalInput Arm_Extension_RetractSlowSwitch = new DigitalInput(1);

  private final SingleJointedArmSim Arm_Rot_Sim =
          new SingleJointedArmSim(
                  DCMotor.getFalcon500(2),
                  Constants_Arm.Sim.ARM_REDUCTION,
                  SingleJointedArmSim.estimateMOI(Constants_Arm.Sim.ARM_LENGTH, Constants_Arm.Sim.ARM_WEIGHT),
                  Constants_Arm.Sim.ARM_LENGTH,
                  Constants_Arm.Sim.ARM_Min_ANGLE,
                  Constants_Arm.Sim.ARM_MAX_ANGLE,
                  true
          );

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d mechanism2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d armPivot = mechanism2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d armTower =
          armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d arm =
          armPivot.append(
                  new MechanismLigament2d(
                          "Arm",
                          30,
                          Units.radiansToDegrees(Arm_Rot_Sim.getAngleRads()),
                          6,
                          new Color8Bit(Color.kYellow)));
  private Boolean isSlowSwitchClosed;

  // private Boolean inSlowArea;

  private Boolean isStopSwitchClosed;
  private Boolean outsideBounds;
  private final double handLength;

  public SubSys_Arm(double handLength) {
    this.handLength = handLength;

    SmartDashboard.putData("Arm Sim", mechanism2d);
    armTower.setColor(new Color8Bit(Color.kBlue));

    configArmShoulderMotor();
    configArmShoulderFollowerMotor();
    configArmShoulderEncoder();

    configArmExtensionMotor();
    configArmExtensionEncoder();

    double armShoulderAngleCorrectedZero = initAbsolutePosOfShoulderCorrectingZeroPoint();
    SmartDashboard.putNumber("armShoulderAngleCorrectedZero", armShoulderAngleCorrectedZero);
    SmartDashboard.putNumber("armShoulderAngleRaw", Arm_ShoulderEncoder.getAbsolutePosition());
    Arm_ShoulderEncoder.setPosition(armShoulderAngleCorrectedZero);

    outsideBounds = checkOutsideBounds();
  }

  private double initAbsolutePosOfShoulderCorrectingZeroPoint () {
    double armAbsShoulderAngleCorrected =
            Arm_ShoulderEncoder.getAbsolutePosition() - Constants_Arm.ARM_SHOULDER_ZERO_ANGLE;

    // Make sure Initial Angle is between 45--225
    if (armAbsShoulderAngleCorrected > 60) {
      armAbsShoulderAngleCorrected -= 360.0;
    }
    return armAbsShoulderAngleCorrected;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Retract Stop Switch Enable", true);
    // Reset ArmExtensionEncoder if RetractStop Switch indicates the ArmExtension is fully
    // retracted.
    if (Constants_Arm.DISABLE_RETRACT_STOP_SWITCH == true) {
      SmartDashboard.putBoolean("Retract Stop Switch Enable", false);
    } else {
      if (isArmReportedRetractedBySwitch()) {
        Arm_ExtensionEncoder.setPosition(0.0);
      }
    }

    outsideBounds = checkOutsideBounds();
    logToSmartDashboard();
  }
  @Override
  public void simulationPeriodic() {
    Arm_ShoulderMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    Arm_ShoulderFollowerMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    Arm_Rot_Sim.update(0.020);

    Arm_ShoulderEncoderSim.setRawPosition((int) Arm_Rot_Sim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(Arm_Rot_Sim.getCurrentDrawAmps()));

    outsideBounds = checkOutsideBounds();
    logToSmartDashboard();
  }


  /***********************************************************************************/
  /* ***** Public Arm Methods *****                                                  */
  /***********************************************************************************/

  /**
   * setArmCmd
   *
   * @param armShoulderRotCmd - 0-1 Percent Command
   * @param armExtensionCmd - 0-1 Percent Command
   */
  public void setArmCmd(double armShoulderRotCmd, double armExtensionCmd) {
    setArmShoulderCmd(armShoulderRotCmd);
    setArmExtensionCmd(armExtensionCmd);
  }

  /**
   * setArmPosCmd
   *
   * @param armShouldPosCmd - Arm Angle in Degrees 45 - 225
   * @param armExtensionPosCmd - ArmHand Length - 31.75 - 64.25
   * @return
   */
  public boolean setArmPosCmd(
      double armShouldPosCmd,
      boolean armShoulderEnable,
      double armExtensionPosCmd,
      boolean armExtensionEnable) {
    boolean ArmShoulderAtSetpoint = setArmShoulderPosCmd(armShouldPosCmd, armShoulderEnable);
    SmartDashboard.putBoolean("ArmShoulderAtSetpoint", ArmShoulderAtSetpoint);
    boolean ArmExtensionAtSetpoint = setArmExtensionPosCmd(armExtensionPosCmd, armExtensionEnable);
    SmartDashboard.putBoolean("ArmExtensionAtSetpoint", ArmExtensionAtSetpoint);
    return (ArmShoulderAtSetpoint && ArmExtensionAtSetpoint);
  }

  // ***** Arm Rotation Methods *****
  public Rotation2d getArmShoulderAngle() {
    return new Rotation2d(Units.degreesToRadians(Arm_ShoulderEncoder.getPosition()));
  }

  // ***** Arm Extension Methods *****

  // Length of Arm and Hand from Arm Shoulder Pivot to Paddle Pivots
  public double getArmHandLength() {
    return getArmLength() + handLength;
  }

  /***********************************************************************************/
  /* ***** Private Arm Methods *****                                                 */
  /***********************************************************************************/

  // ***** Arm Methods *****
  private Translation3d getArmHandTrans3d() {
    return new Translation3d(getArmHandLength(), 0, 0);
  }

  private Rotation3d getArmShoulderRot3d() {
    return new Rotation3d(0, getArmShoulderAngle().getRadians(), 0);
  }

  private Translation3d getArmShoulderAngleArmHandTrans3d() {
    return Robot.Dimensions.Arm.ArmShoulder.plus(
        getArmHandTrans3d().rotateBy(getArmShoulderRot3d()));
  }

  private boolean checkOutsideBounds() {
    boolean outsideBounds = false;
    Translation3d handPosition = getArmShoulderAngleArmHandTrans3d();
    // Check Height
    if (handPosition.getZ() <= Robot.Dimensions.RobotBoundaries.MinHeight) {
      outsideBounds = true;
    } else if (handPosition.getZ() >= Robot.Dimensions.RobotBoundaries.MaxHeight) {
      outsideBounds = true;
    }

    // Check Extension
    if (Math.abs(handPosition.getX())
        >= (Robot.Dimensions.Frame.Length * 0.5
            + Robot.Dimensions.RobotBoundaries.MaxExtensionOverFrame)) {
      outsideBounds = true;
    }

    // Disable bounds if Extension is short
    if (isArmReportedRetractedBySwitch()) {
      outsideBounds = false;
    }
    return outsideBounds;
  }

  // ***** Arm Rotation Methods *****

  // Arm Shoulder Motor Configurations
  private void configArmShoulderMotor() {
    Arm_ShoulderMotor.configFactoryDefault();
    Arm_ShoulderMotor.setInverted(false);
    Arm_ShoulderMotor.setNeutralMode(NeutralMode.Brake);
    Arm_ShoulderMotor.configRemoteFeedbackFilter(Arm_ShoulderEncoder, 0);
    Arm_ShoulderMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    Arm_ShoulderMotor.configSelectedFeedbackCoefficient(1);
    Arm_ShoulderMotor.configOpenloopRamp(Constants_Arm.ArmShoulder.openloopRamp);
    Arm_ShoulderMotor.configClosedloopRamp(Constants_Arm.ArmShoulder.PID.closedLoopRamp);
    Arm_ShoulderMotor.configForwardSoftLimitEnable(
        Constants_Arm.ArmShoulder.ForwardSoftLimitEnable);
    Arm_ShoulderMotor.configForwardSoftLimitThreshold(
        Constants_Arm.ArmShoulder.ForwardSoftLimitThreshold);
    Arm_ShoulderMotor.configReverseSoftLimitEnable(
        Constants_Arm.ArmShoulder.ReverseSoftLimitEnable);
    Arm_ShoulderMotor.configReverseSoftLimitThreshold(
        Constants_Arm.ArmShoulder.ReverseSoftLimitThreshold);
    Arm_ShoulderMotor.configMotionCruiseVelocity(100);
    Arm_ShoulderMotor.configMotionAcceleration(100);
    // Slot 0
    Arm_ShoulderMotor.config_kP(0, Constants_Arm.ArmShoulder.PID.kP);
    Arm_ShoulderMotor.config_kI(0, Constants_Arm.ArmShoulder.PID.kI);
    Arm_ShoulderMotor.config_kD(0, Constants_Arm.ArmShoulder.PID.kD);
    Arm_ShoulderMotor.config_IntegralZone(0, Constants_Arm.ArmShoulder.PID.IntegralZone);
    Arm_ShoulderMotor.configAllowableClosedloopError(
        0, Constants_Arm.ArmShoulder.PID.allowableClosedLoopError);
  }

  private void configArmShoulderFollowerMotor() {
    Arm_ShoulderFollowerMotor.configFactoryDefault();
    Arm_ShoulderFollowerMotor.setInverted(true);
    Arm_ShoulderFollowerMotor.setNeutralMode(NeutralMode.Coast);
    // Arm_ShoulderFollowerMotor.setNeutralMode(NeutralMode.Brake);
    Arm_ShoulderFollowerMotor.follow(Arm_ShoulderMotor);
  }

  /** configArmShoulderEncoder Configure Arm Shoulder Encoder */
  private void configArmShoulderEncoder() {
    Arm_ShoulderEncoder.configFactoryDefault();
    Arm_ShoulderEncoder.configSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition);
    Arm_ShoulderEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    Arm_ShoulderEncoder.configSensorDirection(true);
    /*
    double armShoulderAngle_Init = Arm_ShoulderEncoder.getAbsolutePosition()-SubSys_Arm_Constants.ArmShoulderZeroAngle;

    // Make sure Initial Angle is between 45--225
    if(armShoulderAngle_Init>60){
      armShoulderAngle_Init = armShoulderAngle_Init-360.0;
    //}else if(armShoulderAngle_Init<-230){
      //armShoulderAngle_Init = armShoulderAngle_Init+360;
    }
    SmartDashboard.putNumber("armShoulderAngle_Init", armShoulderAngle_Init);
    Arm_ShoulderEncoder.setPosition(armShoulderAngle_Init);
    */
  }

  private Rotation2d getArmShoulderEncoderAngleAbsolute() {
    return new Rotation2d(Units.degreesToRadians(Arm_ShoulderEncoder.getAbsolutePosition()));
  }

  private Rotation2d getArmShoulderEncoderAngle() {
    return new Rotation2d(Units.degreesToRadians(Arm_ShoulderEncoder.getPosition()));
  }

  private void setArmShoulderCmd(double armShoulderRotCmd) {
    // Convert from rotSpd to Percent
    double armRotCmd = armShoulderRotCmd / Robot.MaxSpeeds.Arm.ArmShoulderMaxRotSpd;
    armRotCmd = armRotCmd + calcArmShoulderFF();

    if (outsideBounds) {
      armRotCmd = 0.0;
    }
    // Check for Mechanical Rotation Limits
    if (getArmShoulderAngle().getDegrees() <= Constants_Arm.ArmShoulderMinAngle) {
      armRotCmd = Math.max(armRotCmd, 0);
    } else if (getArmShoulderAngle().getDegrees() >= Constants_Arm.ArmShoulderMaxAngle) {
      armRotCmd = Math.min(armRotCmd, 0);
    }

    SmartDashboard.putNumber("armRotCmd", armRotCmd);
    Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, armRotCmd);
  }

  private boolean setArmShoulderPosCmd(double armShoulderRotPosCmd, boolean armShoulderEnable) {
    boolean atSetpoint = false;
    if (armShoulderEnable && !outsideBounds) {
      // Check for Mechanical Rotation Limits
      double rotPosCmd =
          Math.min(
              Constants_Arm.ArmShoulderMaxAngle,
              Math.max(armShoulderRotPosCmd, Constants_Arm.ArmShoulderMinAngle));

      // Convert to TalonFX CANCoder Units
      double armShoulderMotPosCmd = TalonFX_Conversions.degreesToCANCoderCnts(rotPosCmd);

      // Arm_ShoulderMotor.set(TalonFXControlMode.Position, rotPosCmd);
      Arm_ShoulderMotor.set(
          TalonFXControlMode.MotionMagic,
          armShoulderMotPosCmd,
          DemandType.ArbitraryFeedForward,
          calcArmShoulderFF());

      atSetpoint =
          ((Math.abs(getArmShoulderAngle().getDegrees() - rotPosCmd))
              < Constants_Arm.ArmShoulder.PID.atSetpointAllowableError);
    } else {
      Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, 0);
      atSetpoint = true;
    }
    return atSetpoint;
  }

  private double calcArmShoulderFF() {
    double FFFactor =
        ((getArmLength() - Robot.Dimensions.Arm.ArmMinLength)
                * (Constants_Arm.ArmShoulder.FF.MaxFFPct
                    - Constants_Arm.ArmShoulder.FF.MinFFPct))
            / (Robot.Dimensions.Arm.ArmMaxExtensionLength - Robot.Dimensions.Arm.ArmMinLength);
    double MaxFFCmd = Constants_Arm.ArmShoulder.FF.MinFFPct + FFFactor;
    double FFCmd = MaxFFCmd * Math.cos(getArmShoulderAngle().getRadians());
    SmartDashboard.putNumber("ArmShoulder_FFCmd", FFCmd);
    return 0.0;
  }

  // ***** Arm Extension Methods *****

  // Configure ArmExtensionMotor
  private void configArmExtensionMotor() {
    Arm_ExtensionMotor.configFactoryDefault();
    Arm_ExtensionMotor.setInverted(true);
    Arm_ExtensionMotor.setNeutralMode(NeutralMode.Brake);
    Arm_ExtensionMotor.configRemoteFeedbackFilter(Arm_ExtensionEncoder, 0);
    Arm_ExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    Arm_ExtensionMotor.configSelectedFeedbackCoefficient(-1);
    // Arm_ExtensionMotor.configSelectedFeedbackCoefficient(7.854 / 4096);

    Arm_ExtensionMotor.configOpenloopRamp(Constants_Arm.ArmExtension.openloopRamp);
    Arm_ExtensionMotor.configClosedloopRamp(Constants_Arm.ArmExtension.PID.closedLoopRamp);
    Arm_ExtensionMotor.configForwardSoftLimitEnable(
        Constants_Arm.ArmExtension.ForwardSoftLimitEnable);
    Arm_ExtensionMotor.configForwardSoftLimitThreshold(
        Constants_Arm.ArmExtension.ForwardSoftLimitThreshold);
    Arm_ExtensionMotor.configReverseSoftLimitEnable(
        Constants_Arm.ArmExtension.ReverseSoftLimitEnable);
    Arm_ExtensionMotor.configReverseSoftLimitThreshold(
        Constants_Arm.ArmExtension.ReverseSoftLimitThreshold);
    // Slot 0
    Arm_ExtensionMotor.config_kP(0, Constants_Arm.ArmExtension.PID.kP);
    Arm_ExtensionMotor.config_kI(0, Constants_Arm.ArmExtension.PID.kI);
    Arm_ExtensionMotor.config_kD(0, Constants_Arm.ArmExtension.PID.kD);
    Arm_ExtensionMotor.configAllowableClosedloopError(
        0, Constants_Arm.ArmExtension .PID.allowableClosedLoopError);
  }

  /** configArmExtensionEncoder */
  private void configArmExtensionEncoder() {
    Arm_ExtensionEncoder.configFactoryDefault();
    Arm_ExtensionEncoder.configSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition);
    Arm_ExtensionEncoder.configSensorDirection(false);
    Arm_ExtensionEncoder.setPosition(0.0);
  }

  /**
   * @return true if the arm is reported to be fully retracted false otherwise
   */
  private boolean isArmReportedRetractedBySwitch() {
    boolean armSwitchStateIndicatesFullRetraction = false;
    if (!Arm_Extension_RetractStopSwitch.get()) {
      armSwitchStateIndicatesFullRetraction = true;
    }
    return armSwitchStateIndicatesFullRetraction;
  }

  private Rotation2d getArmExtensionEncoderAngleAbsolute() {
    return new Rotation2d(Units.degreesToRadians(Arm_ExtensionEncoder.getAbsolutePosition()));
  }

  private Rotation2d getArmExtensionEncoderAngle() {
    return new Rotation2d(Units.degreesToRadians(Arm_ExtensionEncoder.getPosition()));
  }

  // Length of Arm from Arm Shoulder Pivot to Hand Pivot
  private double getArmLength() {
    return Robot.Dimensions.Arm.ArmMinLength + getArmExtensionEncoderLength();
  }

  // Length of ArmExtension Length only (0-ArmMaxExtensionLength)
  private double getArmExtensionEncoderLength() {
    return Arm_ExtensionEncoder.getPosition() * Constants_Arm.ArmExtensionDegToMetersFactor;
  }

  private void setArmExtensionCmd(double armExtensionCmd) {
    double armExtCmd = armExtensionCmd / Robot.MaxSpeeds.Arm.ArmExtensionMaxSpd;

    if (outsideBounds) {
      armExtCmd = -0.3;
    }
    // Check for Mechanical Extension Limits
    double maxExension =
        Constants_Arm.ArmExtensionEncoderFullyExtendedDegrees
            * Constants_Arm.ArmExtensionDegToMetersFactor;
    if (isArmReportedRetractedBySwitch()) {
      armExtCmd = Math.max(armExtCmd, 0);
    } else if (getArmExtensionEncoderLength()
        <= 0.0 + Robot.Calibrations.Arm.ArmExtendPosCtrlSlowRange) {
      armExtCmd = Math.max(armExtCmd, -Robot.Calibrations.Arm.ArmExtendPosCtrlSlowSpd);
    } else if (getArmExtensionEncoderLength()
        >= maxExension - Robot.Calibrations.Arm.ArmExtendPosCtrlSlowRange) {
      armExtCmd = Math.min(armExtCmd, Robot.Calibrations.Arm.ArmExtendPosCtrlSlowSpd);
    } else if (getArmExtensionEncoderLength() >= maxExension) {
      armExtCmd = Math.min(armExtCmd, 0);
    }
    SmartDashboard.putNumber("armExtCmd", armExtCmd);
    Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, armExtCmd);
  }

  private boolean setArmExtensionPosCmd(double armExtensionPosCmd, boolean armExtensionEnable) {
    boolean atPosition = false;
    double extPosCmd = Robot.Dimensions.Arm.ArmMinLength;
    if (armExtensionEnable || outsideBounds) {
      // Check for Mechanical Extension Limits
      if (outsideBounds) {
        extPosCmd = Robot.Dimensions.Arm.ArmMinLength;
      } else {
        extPosCmd = armExtensionPosCmd;
      }
      extPosCmd =
          Math.min(
              Robot.Dimensions.Arm.ArmMaxExtensionLength + Robot.Dimensions.Hand.HandForwardLength,
              Math.max(
                  extPosCmd,
                  Robot.Dimensions.Arm.ArmMinLength + Robot.Dimensions.Hand.HandForwardLength));

      double posError = extPosCmd - getArmHandLength();
      double extMtrCmd = 0;

      if (posError > Robot.Calibrations.Arm.ArmExtendPosCtrlSlowRange) {
        extMtrCmd = Robot.Calibrations.Arm.ArmExtendPosCtrlFastSpd;
        atPosition = false;
      } else if (posError > Robot.Calibrations.Arm.ArmExtendPosCtrlAtPositionRange) {
        extMtrCmd = Robot.Calibrations.Arm.ArmExtendPosCtrlSlowSpd;
        atPosition = false;
      } else if (posError < -Robot.Calibrations.Arm.ArmExtendPosCtrlSlowRange) {
        extMtrCmd = -Robot.Calibrations.Arm.ArmExtendPosCtrlFastSpd;
        atPosition = false;
      } else if (posError < -Robot.Calibrations.Arm.ArmExtendPosCtrlAtPositionRange) {
        extMtrCmd = -Robot.Calibrations.Arm.ArmExtendPosCtrlSlowSpd;
        atPosition = false;
      } else {
        extMtrCmd = 0.0;
        atPosition = true;
      }
      SmartDashboard.putNumber("extPosCmd", extPosCmd);
      SmartDashboard.putNumber("posError", posError);
      SmartDashboard.putNumber("extCmd", extMtrCmd);

      Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, extMtrCmd);
    } else {
      Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, 0);
      atPosition = true;
    }

    SmartDashboard.putBoolean("atPosition", atPosition);
    return atPosition;
  }

  // *Math methods
  // z = height
  public double getHeightOfArmFromBase(double ArmShoulderAngle, double ArmExtensionLength) {
    if (ArmShoulderAngle >= 0 && ArmShoulderAngle <= 180) {
      double radians = Math.toRadians(ArmShoulderAngle); // Convert from degrees to radians
      return Const_Arm.kARM_SHOULDER_z
          + (Math.sin(radians)
              * (ArmExtensionLength + Const_Arm.kHAND_LENGTH)); // if angle is between 0-180
    } else
      return Const_Arm
          .kARM_SHOULDER_z; // if not above the pivot point then hight is the pivot point
  }

  // x = offset
  public double getLengthOfArmFromBase(double ArmShoulderAngle, double ArmExtensionLength) {
    double radians = Math.toRadians(ArmShoulderAngle); // Convert from degrees to radians
    return (Const_Arm.kARM_SHOULDER_x
            + Math.cos(radians) * (ArmExtensionLength + Const_Arm.kHAND_LENGTH))
        - Const_Arm
            .kROBOT_WIDTH; // no if needed because we simply subtract the robots width and it does
    // not matter if this becomes negative
  }

  // *Motor methods (-Basic-)
  /**
   * @param lockValue 0 = Unlocked, 1 = Pos-No-Neg 2, = Neg-No-Pos
   * @param PercentOutput -1 - 1 double
   */
  public void RotateArm(int lockValue, double PercentOutput) {
    switch (lockValue) {
      case 0: // Unlocked
        Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, PercentOutput);
        // SmartDashboard.putString("RotateArmLockValue", "Specified");
        break;
      case 1: // Pos-No-Neg max
        Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, Math.max(0, PercentOutput));
        // SmartDashboard.putString("RotateArmLockValue", "Specified");
        break;
      case 2: // Neg-No-Pos min
        Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, Math.min(0, PercentOutput));
        // SmartDashboard.putString("RotateArmLockValue", "Specified");
        break;
      default:
        // SmartDashboard.putString("RotateArmLockValue", "Not Specified");
        break;
    }
  }

  //   /**
  //    * @param lockValue 0 = Unlocked, 1 = Ext-No-Rtc, 2 = Rtc-No-Ext, 3 = Slow
  //    * @param PercentOutput -1 - 1 double
  //    */
  public void ExtendArm(int lockValue, double PercentOutput) {
    switch (lockValue) {
      case 0: // Unlocked
        // SmartDashboard.putString("ExtendArmLockValue", "Specified");
        Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, PercentOutput);
        break;
      case 1: // Extend No Retract
        //  SmartDashboard.putString("ExtendArmLockValue", "Specified");
        Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, Math.max(0, PercentOutput));
        break;
      case 2: // Retract No Extend
        //  SmartDashboard.putString("ExtendArmLockValue", "Specified");
        Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, Math.min(0, PercentOutput));
        break;
      case 3: // Slow
        //  SmartDashboard.putString("ExtendArmLockValue", "Specified");
        if (PercentOutput > 0) { // Extending is not slowed
          Arm_ExtensionMotor.set(TalonFXControlMode.PercentOutput, PercentOutput);
        }
        if (PercentOutput > 0) { // Retracting is slowed
          Arm_ExtensionMotor.set(
              TalonFXControlMode.PercentOutput, PercentOutput * Const_Arm.kSLOW_MULTIPLIER);
        }
        break;
      default:
        //  SmartDashboard.putString("ExtendArmLockValue", "Not Specified");
        break;
    }
  }

  // *Motor methods (-InBoundsMethods-)
  public void RotateArm_InBounds(double PercentOutput) {
    double ArmShoulderAngle = Arm_ShoulderEncoder.getAbsolutePosition() - Const_Arm.kOffsetTo0;
    double ArmExtendLength = Arm_ExtensionMotor.getSelectedSensorPosition();

    double currentHeight = getHeightOfArmFromBase(ArmShoulderAngle, ArmExtendLength);
    double currentLength = getLengthOfArmFromBase(ArmShoulderAngle, ArmExtendLength);

    if (currentHeight < Const_Arm.kMAX_EXTENSION_z && currentLength < Const_Arm.kMAX_EXTENSION_x) {
      SmartDashboard.putString("Boundary", "No boundary hit");
      Arm_ShoulderMotor.set(TalonFXControlMode.PercentOutput, PercentOutput);
    }

    if (currentHeight > Const_Arm.kMAX_EXTENSION_z) {
      if (ArmShoulderAngle >= 0 && ArmShoulderAngle < 90) RotateArm(2, PercentOutput);
      if (ArmShoulderAngle >= 90 && ArmShoulderAngle < 180) RotateArm(1, PercentOutput);
    }

    if (currentHeight > Const_Arm.kMAX_EXTENSION_x) {
      SmartDashboard.putString("Boundary", "Width boundary hit");
      if (ArmShoulderAngle >= 0 && ArmShoulderAngle < 90) RotateArm(1, PercentOutput);
      if (ArmShoulderAngle >= 90 && ArmShoulderAngle < 180) RotateArm(2, PercentOutput);
      if (ArmShoulderAngle >= 180 && ArmShoulderAngle < 270) RotateArm(1, PercentOutput);
      if (ArmShoulderAngle >= 270 && ArmShoulderAngle < 360) RotateArm(2, PercentOutput);
    }

    if (currentHeight > Const_Arm.kMAX_EXTENSION_z && currentLength > Const_Arm.kMAX_EXTENSION_x) {
      SmartDashboard.putString("Boundary", "Both boundary hit");
      RotateArm(0, 0);
    }
  }

  public void rotateArmMinMax(double percentOutput, double min, double max) {
    double ArmShoulderAngle = Arm_ShoulderEncoder.getAbsolutePosition() - Const_Arm.kOffsetTo0;
    double ArmExtendLength = Arm_ExtensionMotor.getSelectedSensorPosition();

    if (ArmShoulderAngle > max && ArmShoulderAngle < max + 30) {
      RotateArm(1, percentOutput);
    } else if (ArmShoulderAngle < min && ArmShoulderAngle > min - 30) {
      RotateArm(2, percentOutput);
    } else {
      RotateArm(0, percentOutput);
    }
  }

  public void ExtendArm_InBounds(double PercentOutput) {
    double ArmShoulderAngle = Arm_ShoulderEncoder.getAbsolutePosition() - Const_Arm.kOffsetTo0;
    double ArmExtendLength = Arm_ExtensionMotor.getSelectedSensorPosition();

    double currentHeight = getHeightOfArmFromBase(ArmShoulderAngle, ArmExtendLength);
    double currentLength = getLengthOfArmFromBase(ArmShoulderAngle, ArmExtendLength);

    // if (isSlowSwitchClosed || inSlowArea && !isSwitch1Closed) {
    //   ExtendArm(3, PercentOutput);

    //   if (Arm_ExtensionCanCoder.getPosition() > Const_Arm.kSWITCH_ARM_LOCATION) {inSlowArea =
    // false; }
    //   else { inSlowArea = true;}

    //   if (isSlowSwitchClosed) {
    //     Arm_ExtensionCanCoder.setPosition(Const_Arm.kSWITCH_ARM_LOCATION);
    //   }
    // }
    if (isStopSwitchClosed) {
      ExtendArm(1, PercentOutput);
      Arm_ExtensionEncoder.setPosition(0);
    }
    if (!isStopSwitchClosed && !isSlowSwitchClosed) {
      if (currentHeight < Const_Arm.kMAX_EXTENSION_z
          && currentLength < Const_Arm.kMAX_EXTENSION_x) {
        ExtendArm(0, PercentOutput);
      } else {
        ExtendArm(2, PercentOutput);
      }
    }
  }

  /**
   * Arm extension control method that limits the extension of the arm within the specified minimum
   * and maximum values.
   *
   * @param PercentOutput the percentage of output/speed to use for the arm extension motor
   * @param min the minimum allowed extension value for the arm
   * @param max the maximum allowed extension value for the arm
   */
  public void armExtentionMinMax(double PercentOutput, double min, double max) {
    double ArmExtendLength = Arm_ExtensionMotor.getSelectedSensorPosition();

    if (isStopSwitchClosed) {
      ExtendArm(2, PercentOutput);
      Arm_ExtensionEncoder.setPosition(0);
    }
    if (!isStopSwitchClosed) {
      if (ArmExtendLength < max) {
        ExtendArm(0, PercentOutput);
      }
      if (ArmExtendLength > max) {
        ExtendArm(1, PercentOutput);
      }
    }
  }

  public double getShoulderRotation() {
    return Arm_ShoulderEncoder.getAbsolutePosition() - Const_Arm.kOffsetTo0;
  }

  public void resetExtendCanCoder() {
    Arm_ExtensionEncoder.setPosition(0);
  }

  private void logToSmartDashboard() {
    SmartDashboard.putBoolean("outsideBounds", outsideBounds);
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(
            "ArmShoulderEncoderAngleAbsolute", getArmShoulderEncoderAngleAbsolute().getDegrees());
    SmartDashboard.putNumber("ArmShoulderEncoderAngle", getArmShoulderEncoderAngle().getDegrees());
    SmartDashboard.putNumber(
            "Arm_ShoulderMotor_Pos", Arm_ShoulderMotor.getSelectedSensorPosition());

    SmartDashboard.putNumber(
            "ArmExtensionEncoderAngleAbsolute", getArmExtensionEncoderAngleAbsolute().getDegrees());
    SmartDashboard.putNumber(
            "ArmExtensionEncoderAngle", getArmExtensionEncoderAngle().getDegrees());
    SmartDashboard.putNumber(
            "Arm_ExtensionMotor_Pos", Arm_ExtensionMotor.getSelectedSensorPosition());

    SmartDashboard.putBoolean(
            "ArmExtensionFullyRetractSwitchActive", isArmReportedRetractedBySwitch());

    SmartDashboard.putNumber("ArmExtensionEncoderLength", getArmExtensionEncoderLength());
    SmartDashboard.putNumber("ArmLength", getArmLength());
    SmartDashboard.putNumber("handLength", handLength);
    SmartDashboard.putNumber("ArmHandLength", getArmHandLength());

    SmartDashboard.putNumber("ArmHandTrans3d_X", Units.metersToInches(getArmHandTrans3d().getX()));
    SmartDashboard.putNumber("ArmHandTrans3d_Y", Units.metersToInches(getArmHandTrans3d().getY()));
    SmartDashboard.putNumber("ArmHandTrans3d_Z", Units.metersToInches(getArmHandTrans3d().getZ()));

    SmartDashboard.putNumber(
            "ArmShoulderRot3d_X", Units.metersToInches(getArmShoulderRot3d().getX()));
    SmartDashboard.putNumber(
            "ArmShoulderRot3d_Y", Units.metersToInches(getArmShoulderRot3d().getY()));
    SmartDashboard.putNumber(
            "ArmShoulderRot3d_Z", Units.metersToInches(getArmShoulderRot3d().getZ()));

    SmartDashboard.putNumber(
            "ArmShoulderAngleArmHandTrans3d_X",
            Units.metersToInches(getArmShoulderAngleArmHandTrans3d().getX()));
    SmartDashboard.putNumber(
            "ArmShoulderAngleArmHandTrans3d_Y",
            Units.metersToInches(getArmShoulderAngleArmHandTrans3d().getY()));
    SmartDashboard.putNumber(
            "ArmShoulderAngleArmHandTrans3d_Z",
            Units.metersToInches(getArmShoulderAngleArmHandTrans3d().getZ()));
  }
}
