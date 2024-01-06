// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* ===========================================================================*/
/* Swerve and Steer Falcon Drive and PG71 Steer                               */
/*                                                                            */
/* Drive Motor - Falcon Motor                                                 */
/* Drive Controller - TalonFX                                                 */
/* Drive Sensor - Falcon Motor                                                */
/* Drive Control Method - FalconFX
/* Steer Motor - PG71 Motor                                                   */
/* Steer Controller - TalonSRX                                                */
/* Steer Sensor - MA3                                                         */
/* Steer Control Method - TalonSRX                                            */
/*                                                                            */
/* ===========================================================================*/

package frc.robot.library.drivetrains.swervedrive.swervemodules.swerveandsteer_falcon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.library.drivetrains.SubSys_DriveTrain_Constants;
import frc.robot.library.drivetrains.swervedrive.swervemodules.CTREModuleState;
import frc.robot.library.drivetrains.swervedrive.swervemodules.SwerveModuleConstants;
import frc.robot.library.drivetrains.swervedrive.swervemodules.swerveandsteer_falcon.SwrNStr_FalconPG71_Module_Constants.DriveMotor;
import frc.robot.library.motorcontrollers.talonfx.TalonFX_Conversions;
import frc.robot.library.motorcontrollers.talonsrx.TalonSRX_Conversions;

/** Add your docs here. */
public class SwrNStr_FalconPG71_Module {

  public String moduleName;
  private TalonFX driveMotor;
  private TalonSRX steerMotor;
  // private double steerAngleOffset;
  private double steerZeroAngle;
  private double lastAngle;
  private SwrNStr_FalconPG71_Module_Constants swrNStr_FalconPG71_Module_Constants;

  SimpleMotorFeedforward driveMotorFF =
      new SimpleMotorFeedforward(DriveMotor.driveKS, DriveMotor.driveKV, DriveMotor.driveKA);

  public SwrNStr_FalconPG71_Module(String moduleName, SwerveModuleConstants moduleConstants) {
    this.moduleName = moduleName;
    this.steerZeroAngle = moduleConstants.zeroAngle;
    this.swrNStr_FalconPG71_Module_Constants = new SwrNStr_FalconPG71_Module_Constants();

    /* Drive Motor Config */
    this.driveMotor = new TalonFX(moduleConstants.driveMotorID);
    configDriveMotor(moduleConstants);

    /* Steer Motor Config */
    this.steerMotor = new TalonSRX(moduleConstants.steerMotorID);
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

    /** Drive Motor Command */
    // Open Loop
    if (isOpenLoop) {
      double percentOutput =
          desiredState.speedMetersPerSecond / SubSys_DriveTrain_Constants.DriveTrainMaxSpd;
      driveMotor.set(ControlMode.PercentOutput, percentOutput);
    }
    // Closed Loop Velocity (PID Slot 0)
    else {

      driveMotor.set(
          ControlMode.Velocity,
          convertMPSToDriveMtrCntsPer100ms(desiredState.speedMetersPerSecond),
          DemandType.ArbitraryFeedForward,
          driveMotorFF.calculate(desiredState.speedMetersPerSecond));
    }

    /**
     * Steer Motor Command Always Closed Loop If Drive Speed Command is less than 1%, do not rotate
     * to prevent jittering
     */

    // Set default angle to lastAngle
    double angle = lastAngle;
    if (Math.abs(desiredState.speedMetersPerSecond)
        > (SubSys_DriveTrain_Constants.DriveTrainMaxSpd * 0.01)) {
      angle = desiredState.angle.getDegrees();
    }

    // Set Steet Motor Command to angle
    steerMotor.set(
        ControlMode.Position,
        TalonSRX_Conversions.degreesToMA3(
            angle, SwrNStr_FalconPG71_Module_Constants.SteerMotor.steerGearRatio));
    // Update last angle
    lastAngle = angle;
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
    driveMotor.configAllSettings(SwrNStr_FalconPG71_Module_Constants.DriveTalonFXConfig);
    driveMotor.setInverted(moduleConstants.driveMotorInvert);
    driveMotor.setNeutralMode(SwrNStr_FalconPG71_Module_Constants.DriveMotor.neutralMode);
    driveMotor.setSelectedSensorPosition(0);
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
        * SwrNStr_FalconPG71_Module_Constants.DriveMotor.invDriveGearRatio;
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
        * SwrNStr_FalconPG71_Module_Constants.DriveMotor.driveWheelCircumference;
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
        * SwrNStr_FalconPG71_Module_Constants.DriveMotor.invDriveGearRatio
        * SwrNStr_FalconPG71_Module_Constants.DriveMotor.driveWheelCircumference
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
            * SwrNStr_FalconPG71_Module_Constants.DriveMotor.invDriveWheelCircumference
            * SwrNStr_FalconPG71_Module_Constants.DriveMotor.driveGearRatio);
  }

  /***********************************************************************************/
  /* ***** Steer Angle Encoder Methods *****                                         */
  /***********************************************************************************/

  // Analog MA3 Steer Angle Encoder

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
    steerMotor.configAllSettings(SwrNStr_FalconPG71_Module_Constants.SteerTalonSRXConfig);
    steerMotor.setInverted(moduleConstants.steerMotorInvert);
    steerMotor.setNeutralMode(SwrNStr_FalconPG71_Module_Constants.SteerMotor.neutralMode);

    steerMotor.configSelectedFeedbackSensor(
        SwrNStr_FalconPG71_Module_Constants.SteerMotor.selectedFeedbackSensor);
    steerMotor.configSelectedFeedbackCoefficient(
        SwrNStr_FalconPG71_Module_Constants.SteerMotor.selectedFeedbackCoefficient);
    steerMotor.configFeedbackNotContinuous(
        SwrNStr_FalconPG71_Module_Constants.SteerMotor.feedbackNotContinuous, 0);
    steerMotor.setSensorPhase(moduleConstants.SteerCANcoderInvert);
  }

  // Works Good!
  /**
   * getSteerMotorPosCounts Return Steer Motor Position Counts
   *
   * @return double Steer Motor Position (Raw sensor units)
   */
  public double getSteerMotorPosCounts() {
    return steerMotor.getSelectedSensorPosition();
  }

  public double getSteerAngleRaw() {
    double strAngRaw = TalonSRX_Conversions.ma3ToDegrees(steerMotor.getSelectedSensorPosition(), 1);
    return strAngRaw;
  }

  /**
   * getSteerAngle Return Steer Motor Angle in Rotation2d
   *
   * @return Rotation2d Steer Motor Angle
   */
  public Rotation2d getSteerAngle() {
    Rotation2d strAng = Rotation2d.fromDegrees(getSteerAngleRaw() - this.steerZeroAngle);
    return strAng;
  }

  public double steerAngleToSteerAngleRaw(Rotation2d strAng) {
    double strAngRaw = strAng.getDegrees() + this.steerZeroAngle;
    return strAngRaw;
  }

  public double steerAngleRawToMotorPosCounts(double strAngRaw) {
    double strMtrPosCnts = TalonSRX_Conversions.degreesToMA3(strAngRaw, 1);
    return strMtrPosCnts;
  }

  /***********************************************************************************/
  /* ***** Are These Needed? *****                                                   */
  /***********************************************************************************/

  /**
   * getSteerSensorAbsolutePosCnts Returns the Absolute Position Measurement Cnts
   *
   * @return double Absolute Steer Sensor Positions Measurement (Counts)
   */
  public double getSteerSensorAbsolutePosCnts() {
    return steerMotor.getSelectedSensorPosition();
  }

  /**
   * getSteerSensorAbsolutePos Returns the Absolute Position Measurement
   *
   * @return double Absolute Steer Sensor Positions Measurement (Degrees)
   */
  public double getSteerSensorAbsolutePos() {
    // TalonSRX_Conversions.ma3ToDegrees(
    //    steerSensorCntsCorrected(),
    //    SwrNStr_FalconPG71_Module_Constants.SteerMotor.steerGearRatio);
    return getSteerSensorAbsolutePosCnts() * 360 / 1024;
  }

  /**
   * getSteerSensorPos Returns the Position Measurement
   *
   * @return double Steer Sensor Position Measurement
   */
  public double getSteerSensorPos() {
    return 0; // steerAngleEncoder.getPosition();
  }

  public double steerSensorCnts() {
    double mSteerSensorCnts = steerMotor.getSelectedSensorPosition();
    return mSteerSensorCnts;
  }

  public double steerSensorCntsCorrected() {
    double mSteerSensorCntsCorrected = steerSensorCnts() - this.steerZeroAngle;
    return mSteerSensorCntsCorrected;
  }
}
