// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* ===========================================================================*/
/* Swerve and Steer Falcon Drive and PG71 Steer                               */
/*                                                                            */
/* Drive Motor - Falcon Motor                                                 */
/* Drive Controller - TalonFX                                                 */
/* Drive Sensor - Falcon Motor                                                */
/* Drive Control Method - FalconFX                                            */
/* Steer Motor - Falcon Motor                                                 */
/* Steer Controller - TalonFX                                                 */
/* Steer Sensor - CANCoder                                                    */
/* Steer Control Method - TalonFX                                             */
/*                                                                            */
/* ===========================================================================*/

package frc.robot.library.drivetrains.swervedrive.swervemodules.mk4i_falcon;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Robot;
import frc.robot.library.drivetrains.swervedrive.swervemodules.CTREModuleState;
import frc.robot.library.drivetrains.swervedrive.swervemodules.mk4i_falcon.MK4i_FalconFalcon_Module_Constants.*;
import frc.robot.library.drivetrains.swervedrive.swervemodules.SwerveModuleConstants;
import frc.robot.library.motorcontrollers.talonfx.TalonFX_Conversions;

/** Add your docs here. */
public class MK4i_FalconFalcon_Module {

  public String moduleName;
  public TalonFX driveMotor;
  private TalonFX steerMotor;
  private CANCoder steerAngleEncoder;
  private double steerZeroAngle;
  private double lastAngle;
  private MK4i_FalconFalcon_Module_Constants mk4i_Module_Constants;
  private double driveMotorOutputPctCmd = 999;

  SimpleMotorFeedforward driveMotorFF =
      new SimpleMotorFeedforward(DriveMotor.driveKS, DriveMotor.driveKV, DriveMotor.driveKA);

  /**
   * MK4i_FalconFalcon_Module Constructor
   *
   * @param moduleName String Name of Module FL,FR,BL,BR
   * @param moduleConstants SwerveModuleConstants Contains CAN ID's and Basic Motor Settings
   */
  public MK4i_FalconFalcon_Module(String moduleName, SwerveModuleConstants moduleConstants) {
    this.moduleName = moduleName;
    this.steerZeroAngle = moduleConstants.zeroAngle;
    this.mk4i_Module_Constants = new MK4i_FalconFalcon_Module_Constants();

    /* Drive Motor Config */
    this.driveMotor = new TalonFX(moduleConstants.driveMotorID);
    configDriveMotor(moduleConstants);

    /* Angle Encoder Config */
    this.steerAngleEncoder = new CANCoder(moduleConstants.cancoderID);
    configSteerAngleEncoder(moduleConstants);

    /* Steer Motor Config */
    this.steerMotor = new TalonFX(moduleConstants.steerMotorID);
    configSteerMotor(moduleConstants);

    lastAngle = getState().angle.getDegrees();
  }

  /***********************************************************************************/
  /* ***** Swerve Module Methods *****                                               */
  /***********************************************************************************/

  /**
   * setDesiredState Sets the desired state of the Swerve Module
   *
   * @param desiredState SwerveModuleState Desired state
   * @param isOpenLoop boolean Open Loop control for Drive
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState =
        CTREModuleState.optimize(
            desiredState,
            getState().angle); // Custom optimize command, since default WPILib optimize assumes
    // continuous controller which CTRE is not

    /* Drive Motor Command */
    // Open Loop
    if (isOpenLoop) {
      double percentOutput =
          desiredState.speedMetersPerSecond / Robot.MaxSpeeds.DriveTrain.DriveTrainMaxSpd;
      percentOutput = Util.cap(percentOutput, 1);
      driveMotor.set(ControlMode.PercentOutput, percentOutput);
      driveMotorOutputPctCmd = percentOutput;
    }
    // Closed Loop Velocity (PID Slot 0)
    else {
      driveMotor.set(
          ControlMode.Velocity,
          convertMPSToDriveMtrCntsPer100ms(desiredState.speedMetersPerSecond),
          DemandType.ArbitraryFeedForward,
          driveMotorFF.calculate(desiredState.speedMetersPerSecond));
    }

    /* Steer Motor Command
     * Always Closed Loop
     * If Drive Speed Command is less than 1%, do not rotate to prevent jittering
     */

    // Set default angle to lastAngle
    double angle = lastAngle;
    if (Math.abs(desiredState.speedMetersPerSecond)
        > (Robot.MaxSpeeds.DriveTrain.DriveTrainMaxSpd * 0.01)) {
      angle = desiredState.angle.getDegrees();
      // Set Steet Motor Command to angle
      steerMotor.set(ControlMode.Position, TalonFX_Conversions.degreesToCANCoderCnts(angle));
      // Update last angle
      lastAngle = angle;
    } else {
      steerMotor.set(ControlMode.PercentOutput, 0.0);
    }
  }

  /**
   * getState Gets the current state of the module
   *
   * @return SwerveModuleState State of the Module
   */
  public SwerveModuleState getState() {
    /** Drive Wheel Velocity */
    double velocity = convertDriveMtrCntsPer100msToMPS(driveMotor.getSelectedSensorVelocity());

    /** Steer Motor Angle */
    Rotation2d angle = getSteerAngle();
    return new SwerveModuleState(velocity, angle);
  }

  /**
   * Serve Module Position Returns the current position of the module
   *
   * @return ServeModulePosition: The current position of the module
   */
  public SwerveModulePosition getPosition() {
    SwerveModulePosition swrModulePosition;

    swrModulePosition = new SwerveModulePosition(getDriveWheelDistance(), getSteerAngle());

    return swrModulePosition;
  }

  /***********************************************************************************/
  /* ***** Drive Motor Methods *****                                                 */
  /***********************************************************************************/

  /**
   * configDriveMotor Configurations for the Drive Motor are defined in
   * SubSys_SwerveDrive_Constants.java This is basic information like Motor and Sensor CAN ID's,
   * Inverted and Steer Zero Angle
   *
   * @param moduleConstants Module Constants
   */
  private void configDriveMotor(SwerveModuleConstants moduleConstants) {
    driveMotor.configFactoryDefault();
    driveMotor.configAllSettings(this.mk4i_Module_Constants.getDriveMotorConfig());
    driveMotor.setInverted(moduleConstants.driveMotorInvert);
    driveMotor.setNeutralMode(MK4i_FalconFalcon_Module_Constants.DriveMotor.neutralMode);
  }

  /**
   * getDriveMotorSensorPosition Get Drive Motor Sensor Position
   *
   * @return double Integrated Sensor Counts
   */
  public double getDriveMotorSensorPosition() {
    return driveMotor.getSelectedSensorPosition();
  }

  /**
   * getDriveWheelRevs Get Drive Wheel Revolutions
   *
   * @return double Drive Wheel Revolutions
   */
  public double getDriveWheelRevs() {
    // double driveMotorRevs = TalonFX_Conversions.talonFXCntsToRevs(getDriveMotorSensorPosition());
    // double driveWheelRevs =
    // driveMotorRevs*MK4i_FalconFalcon_Module_Constants.DriveMotor.invDriveGearRatio;
    // return driveWheelRevs;

    // More efficient
    return TalonFX_Conversions.talonFXCntsToRevs(getDriveMotorSensorPosition())
        * MK4i_FalconFalcon_Module_Constants.DriveMotor.invDriveGearRatio;
  }

  /**
   * getDriveWheelDistance Get Drive Wheel Distance
   *
   * @return double Drive Wheel Distance (m)
   */
  public double getDriveWheelDistance() {
    // double driveWheelRevs = getDriveWheelRevs();
    // double driveWheelDistance =
    // driveWheelRevs*MK4i_FalconFalcon_Module_Constants.DriveMotor.driveWheelCircumference;
    // return driveWheelDistance;

    // More efficient
    return getDriveWheelRevs()
        * MK4i_FalconFalcon_Module_Constants.DriveMotor.driveWheelCircumference;
  }

  /**
   * setDriveMotorSensorPosition Sets Drive Motor Sensor Position in counts
   *
   * @param counts double Counts
   */
  public void setDriveMotorPos(double counts) {
    driveMotor.setSelectedSensorPosition(counts, 0, 0);
  }

  /**
   * convertDriveMtrCntsPer100msToMPS Convert Drive Motor Counts Per 100ms to Meters Per Second
   *
   * @param velocitycounts double Drive Motor Counts per 100ms
   * @return Velocity in Meters Per Second (mps)
   */
  public static double convertDriveMtrCntsPer100msToMPS(double velocitycounts) {
    // double motorRPM = TalonFX_Conversions.talonFXCntsPer100msToRPM(velocitycounts);
    // double wheelRPM = motorRPM*MK4i_FalconFalcon_Module_Constants.DriveMotor.invDriveGearRatio;
    // double wheelMPS = (wheelRPM *
    // MK4i_FalconFalcon_Module_Constants.DriveMotor.driveWheelCircumference) / 60;
    // return wheelMPS;

    // Mathmatically more efficient
    // 1/60 = 0.0166666666666666666666666666667
    return TalonFX_Conversions.talonFXCntsPer100msToRPM(velocitycounts)
        * MK4i_FalconFalcon_Module_Constants.DriveMotor.invDriveGearRatio
        * MK4i_FalconFalcon_Module_Constants.DriveMotor.driveWheelCircumference
        * 0.016667;
  }

  /**
   * convertMPSToDriveMtrCntsPer100ms Convert Meters Per Second to Drive Motor Counts Per 100ms
   *
   * @param velocity double Velocity in Meters Per Second
   * @return Velocity in Drive Motor Counts Per 100ms
   */
  public static double convertMPSToDriveMtrCntsPer100ms(double velocity) {
    // double wheelRPM =
    // ((velocity*60)/MK4i_FalconFalcon_Module_Constants.DriveMotor.driveWheelCircumference);
    // double motorRPM = wheelRPM*MK4i_FalconFalcon_Module_Constants.DriveMotor.driveGearRatio;
    // double motorVelocityCounts = TalonFX_Conversions.rpmToTalonFXCntsPer100ms(motorRPM);
    // return motorVelocityCounts;

    // Mathmatically more efficient
    // 1/MK4i_FalconFalcon_Module_Constants.DriveMotor.driveWheelCircumference ==
    // MK4i_FalconFalcon_Module_Constants.DriveMotor.invDriveWheelCircumference
    return TalonFX_Conversions.rpmToTalonFXCntsPer100ms(
        velocity
            * 60
            * MK4i_FalconFalcon_Module_Constants.DriveMotor.invDriveWheelCircumference
            * MK4i_FalconFalcon_Module_Constants.DriveMotor.driveGearRatio);
  }

  public double getDriveMotorOutputPctCmd() {
    return driveMotorOutputPctCmd;
  }

  /***********************************************************************************/
  /* ***** Steer Angle Encoder Methods *****                                         */
  /***********************************************************************************/

  /**
   * configSteerAngleEncoder Configure Steer Angle Encoder
   *
   * @param moduleConstants
   */
  private void configSteerAngleEncoder(SwerveModuleConstants moduleConstants) {
    steerAngleEncoder.configSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition);
    steerAngleEncoder.configSensorDirection(moduleConstants.SteerCANcoderInvert);
    double strAngleInit = steerAngleEncoder.getAbsolutePosition() - this.steerZeroAngle;
    SmartDashboard.putNumber("strAngleInit", strAngleInit);
    // steerAngleEncoder.setPosition((steerAngleEncoder.getAbsolutePosition()-this.steerZeroAngle));
    steerAngleEncoder.setPosition(strAngleInit);
  }

  /**
   * getSteerSensorAbsPosDegrees Returns the Absolute Position Measurement in Degrees
   *
   * @return double Absolute Steer Sensor Positions Measurement (Degrees)
   */
  public double getSteerSensorAbsPosDegrees() {
    return steerAngleEncoder.getAbsolutePosition();
  }

  /**
   * getSteerSensorPosDegrees Returns the Position Measurement
   *
   * @return double Steer Sensor Position Measurement
   */
  public double getSteerSensorPosDegrees() {
    return steerAngleEncoder.getPosition();
  }

  /***********************************************************************************/
  /* ***** Steer Motor Methods *****                                                 */
  /***********************************************************************************/

  /**
   * configSteerMotor Config Steer Motor
   *
   * @param moduleConstants
   */
  private void configSteerMotor(SwerveModuleConstants moduleConstants) {
    steerMotor.configFactoryDefault();
    steerMotor.configAllSettings(this.mk4i_Module_Constants.getSteerMotorConfig());
    steerMotor.setInverted(moduleConstants.steerMotorInvert);
    steerMotor.setNeutralMode(MK4i_FalconFalcon_Module_Constants.SteerMotor.neutralMode);

    steerMotor.configSelectedFeedbackSensor(
        MK4i_FalconFalcon_Module_Constants.SteerMotor.selectedFeedbackSensor);
    steerMotor.configSelectedFeedbackCoefficient(
        MK4i_FalconFalcon_Module_Constants.SteerMotor.selectedFeedbackCoefficient);
    steerMotor.configFeedbackNotContinuous(
        MK4i_FalconFalcon_Module_Constants.SteerMotor.feedbackNotContinuous, 0);
    steerMotor.setSensorPhase(MK4i_FalconFalcon_Module_Constants.SteerMotor.sensorPhase);

    // Config Internal Encoder
    steerMotor.configIntegratedSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition);
    steerMotor.configIntegratedSensorOffset(0.0);

    // Config Remote Encoder
    steerMotor.configRemoteFeedbackFilter(steerAngleEncoder, 0);
    steerMotor.setSensorPhase(MK4i_FalconFalcon_Module_Constants.SteerCANcoder.sensorPhase);
    steerMotor.configSelectedFeedbackCoefficient(
        MK4i_FalconFalcon_Module_Constants.SteerCANcoder.selectedFeedbackCoefficient);

    steerMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 0);

    steerMotor.config_kP(0, 0.6, 0);
  }

  /**
   * getSteerMotorPosCounts Return Steer Motor Position Counts
   *
   * @return double Steer Motor Position (Raw sensor units)
   */
  public double getSteerMotorPosCounts() {
    return steerMotor.getSelectedSensorPosition();
  }

  /**
   * getSteerAngle Return Steer Motor Angle in Rotation2d
   *
   * @return Rotation2d Steer Motor Angle
   */
  public Rotation2d getSteerAngle() {
    // double steerAngleDegs = TalonFX_Conversions.canCoderCntsToDegrees(getSteerMotorPosCounts());
    // double steerAngleRADS = Units.degreesToRadians(steerAngleDegs);
    // Rotation2d steerAngle = new Rotation2d(steerAngleRADS);
    // return steerAngle;

    // More efficient
    return new Rotation2d(
        Units.degreesToRadians(
            TalonFX_Conversions.canCoderCntsToDegrees(getSteerMotorPosCounts())));
  }
}
