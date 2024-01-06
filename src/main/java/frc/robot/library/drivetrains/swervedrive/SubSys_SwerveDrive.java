// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library.drivetrains.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Robot;
import frc.robot.library.drivetrains.swervedrive.swervemodules.mk4i_falcon.MK4i_FalconFalcon_Module;
import frc.robot.library.gyroscopes.pigeon2.SubSys_PigeonGyro;

public class SubSys_SwerveDrive extends SubsystemBase {
  /** Creates a new SubSys_SwerveDrive. */

  /* ***** MK4i Swerve *****                                                         */
  public MK4i_FalconFalcon_Module[] swerveModules;

  /* ***** SwrNStr_FalconPG71 Swerve *****                                           */
  // public SwrNStr_FalconPG71_Module[] swerveModules;

  public SwerveDriveOdometry swerveOdometry;
  private SwerveModulePosition[] swerveModulePositionsInit;
  private SubSys_PigeonGyro gyroSubSys;
  // Rotate About Point
  private Translation2d rotationPt;
  private boolean rotateLeftPtCmd_prev;
  private boolean rotateRightPtCmd_prev;

  /**
   * SubSys_SwerveDrive Swerve Drive Subsystem Constructor
   *
   * @param gyroSubSys Gyroscope Subsystem
   */
  public SubSys_SwerveDrive(SubSys_PigeonGyro gyroSubSys) {
    this.gyroSubSys = gyroSubSys;

    this.swerveModulePositionsInit =
        new SwerveModulePosition[] {
          new SwerveModulePosition(0.0, new Rotation2d(0.0)),
          new SwerveModulePosition(0.0, new Rotation2d(0.0)),
          new SwerveModulePosition(0.0, new Rotation2d(0.0)),
          new SwerveModulePosition(0.0, new Rotation2d(0.0))
        };

    // Odometry class for tracking robot pose
    this.swerveOdometry =
        new SwerveDriveOdometry(
            SubSys_SwerveDrive_Constants.swerveKinematics,
            getHeading(),
            this.swerveModulePositionsInit);

    /***********************************************************************************/
    /* ***** MK4i Swerve *****                                                         */
    /***********************************************************************************/

    this.swerveModules =
        new MK4i_FalconFalcon_Module[] {
          new MK4i_FalconFalcon_Module("FL", SubSys_SwerveDrive_Constants.FL_constants),
          new MK4i_FalconFalcon_Module("FR", SubSys_SwerveDrive_Constants.FR_constants),
          new MK4i_FalconFalcon_Module("BL", SubSys_SwerveDrive_Constants.BL_constants),
          new MK4i_FalconFalcon_Module("BR", SubSys_SwerveDrive_Constants.BR_constants)
        };

    /***********************************************************************************/
    /* ***** SwrNStr_FalconPG71 Swerve *****                                           */
    /***********************************************************************************/
    /*
    this.swerveModules = new SwrNStr_FalconPG71_Module[] {
        new SwrNStr_FalconPG71_Module("FL", SubSys_SwerveDrive_Constants.FL_constants),
        new SwrNStr_FalconPG71_Module("FR", SubSys_SwerveDrive_Constants.FR_constants),
        new SwrNStr_FalconPG71_Module("BL", SubSys_SwerveDrive_Constants.BL_constants),
        new SwrNStr_FalconPG71_Module("BR", SubSys_SwerveDrive_Constants.BR_constants)
    };
    */

    this.swerveOdometry.resetPosition(
        new Rotation2d(), this.swerveModulePositionsInit, new Pose2d());

    this.rotationPt = new Translation2d(0, 0);

    this.rotateLeftPtCmd_prev = false;
    this.rotateRightPtCmd_prev = false;
  }

  @Override
  public void periodic() {
    SwerveModulePosition[] swerveModulePositions = getSwerveModulePositions();

    SmartDashboard.putNumber("Odometry_getHeading", getHeading().getDegrees());
    SmartDashboard.putNumber("Odometry_getYawAngle", gyroSubSys.getYawAngle());
    SmartDashboard.putNumber("Odometry_FL_getDegrees", swerveModulePositions[0].angle.getDegrees());
    SmartDashboard.putNumber("Odometry_FL_distanceMeters", swerveModulePositions[0].distanceMeters);
    SmartDashboard.putNumber("Odometry_FR_getDegrees", swerveModulePositions[1].angle.getDegrees());
    SmartDashboard.putNumber("Odometry_FR_distanceMeters", swerveModulePositions[1].distanceMeters);
    SmartDashboard.putNumber("Odometry_BL_getDegrees", swerveModulePositions[2].angle.getDegrees());
    SmartDashboard.putNumber("Odometry_BL_distanceMeters", swerveModulePositions[2].distanceMeters);
    SmartDashboard.putNumber("Odometry_BR_getDegrees", swerveModulePositions[3].angle.getDegrees());
    SmartDashboard.putNumber("Odometry_BR_distanceMeters", swerveModulePositions[3].distanceMeters);

    swerveOdometry.update(getHeading(), swerveModulePositions);

    SmartDashboard.putNumber("Odometry_getPoseMeters_getX", swerveOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry_getPoseMeters_getY", swerveOdometry.getPoseMeters().getY());
    SmartDashboard.putNumber(
        "Odometry_getRotation_getDegrees",
        swerveOdometry.getPoseMeters().getRotation().getDegrees());
    // Front Left (Module 0)

    SmartDashboard.putNumber("FL_SteerMotorPosCounts", swerveModules[0].getSteerMotorPosCounts());
    // SmartDashboard.putNumber("FL_SteerAngleRaw",swerveModules[0].getSteerAngleRaw());
    SmartDashboard.putNumber("FL_SteerAngle", swerveModules[0].getSteerAngle().getDegrees());

    Rotation2d FL_SteerAngle = swerveModules[0].getSteerAngle();
    // double FL_SteerAngleRaw = swerveModules[0].steerAngleToSteerAngleRaw(FL_SteerAngle);
    // double FL_SteerMotorPosCounts =
    // swerveModules[0].steerAngleRawToMotorPosCounts(FL_SteerAngleRaw);
    // SmartDashboard.putNumber("FL_calcSteerAngleRaw", FL_SteerAngleRaw);
    // SmartDashboard.putNumber("FL_calcSteerMotorPosCounts", FL_SteerMotorPosCounts);
    SmartDashboard.putNumber(
        "FL_DriveMtr_OutputPctCmd", swerveModules[0].getDriveMotorOutputPctCmd());
    SmartDashboard.putNumber(
        "FL_DriveMtr_Output_Pct", swerveModules[0].driveMotor.getMotorOutputPercent());

    // SmartDashboard.putNumber("FL_SteerAngleCorrected",swerveModules[0].getSteerAngle().getDegrees());
    // SmartDashboard.putNumber("FL_SteerMotor_Pos", swerveModules[0].getSteerMotorPos());
    // SmartDashboard.putNumber("FL_SteerAngle_Deg", swerveModules[0].getSteerAngle().getDegrees());
    // SmartDashboard.putNumber("FL_DriveMotorSensorPosition",
    // swerveModules[0].getDriveMotorSensorPosition());
    // SmartDashboard.putNumber("FL_DriveWheelRevs", swerveModules[0].getDriveWheelRevs());
    // SmartDashboard.putNumber("FL_DriveWheelDistance", swerveModules[0].getDriveWheelDistance());

    // SmartDashboard.putNumber("FL_SwrPosition_Distance",swerveModules[0].getPosition().distanceMeters);
    // SmartDashboard.putNumber("FL_SwrPosition_Angle",swerveModules[0].getPosition().angle.getDegrees());

    // Front Right (Module 1)
    // SmartDashboard.putNumber("FR_SteerSensor_AbsPosCnts",
    // swerveModules[1].getSteerSensorAbsolutePosCnts());
    // SmartDashboard.putNumber("FR_SteerSensor_AbsPos",
    // swerveModules[1].getSteerSensorAbsolutePos());
    // SmartDashboard.putNumber("FR_SteerSensor_Pos", swerveModules[1].getSteerSensorPos());
    // SmartDashboard.putNumber("FR_SteerMotor_Pos", swerveModules[1].getSteerMotorPos());
    SmartDashboard.putNumber("FR_SteerMotor_Angle", swerveModules[1].getSteerAngle().getDegrees());
    SmartDashboard.putNumber("FR_DriveSensor_Pos", swerveModules[1].getDriveMotorSensorPosition());

    // Back Left (Module 2)
    // SmartDashboard.putNumber("BL_SteerSensor_AbsPosCnts",
    // swerveModules[2].getSteerSensorAbsolutePosCnts());
    // SmartDashboard.putNumber("BL_SteerSensor_AbsPos",
    // swerveModules[2].getSteerSensorAbsolutePos());
    // SmartDashboard.putNumber("BL_SteerSensor_Pos", swerveModules[2].getSteerSensorPos());
    // martDashboard.putNumber("BL_SteerMotor_Pos", swerveModules[2].getSteerMotorPos());
    // SmartDashboard.putNumber("BL_SteerMotor_Angle",
    // swerveModules[2].getSteerAngle().getDegrees());
    SmartDashboard.putNumber("BL_DriveSensor_Pos", swerveModules[2].getDriveMotorSensorPosition());

    // Back Right (Module 3)
    // SmartDashboard.putNumber("BR_SteerSensor_AbsPosCnts",
    // swerveModules[3].getSteerSensorAbsolutePosCnts());
    // SmartDashboard.putNumber("BR_SteerSensor_AbsPos",
    // swerveModules[3].getSteerSensorAbsolutePos());

    // SmartDashboard.putNumber("BR_SteerSensor_Pos", swerveModules[3].getSteerSensorPos());
    // SmartDashboard.putNumber("BR_SteerMotor_Pos", swerveModules[3].getSteerMotorPos());
    SmartDashboard.putNumber("BR_SteerMotor_Angle", swerveModules[3].getSteerAngle().getDegrees());
    SmartDashboard.putNumber("BR_DriveSensor_Pos", swerveModules[3].getDriveMotorSensorPosition());

    // Odometry
    // SmartDashboard.putNumber("Xdistance", swerveOdometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("Ydistance", swerveOdometry.getPoseMeters().getY());
  }

  /***********************************************************************************/
  /* ***** Public Swerve Methods *****                                               */
  /***********************************************************************************/

  // ***** Swerve Drive Methods *****

  /**
   * Drive Command
   *
   * @param translation Translation2d X and Y Robot Velocities in m/s
   * @param rotation Double Rotational Velocity in rads/s
   * @param fieldRelative Boolean Field Relative
   * @param isOpenLoop Boolean Open Loop Velocity Control
   * @param rotateLeftPtCmd Boolean Rotate around Left Point Cmd
   * @param rotateRightPtCmd Boolean Rotate around Right Point Cmd
   */
  public void drive(
      Translation2d translation,
      double rotation,
      boolean fieldRelative,
      boolean isOpenLoop,
      boolean rotateLeftPtCmd,
      boolean rotateRightPtCmd) {

    isOpenLoop = true;

    // Determine Rotation Point
    rotationPt =
        SwerveRotatePointLogic.calcRotationPt(
            rotateLeftPtCmd,
            rotateRightPtCmd,
            rotateLeftPtCmd_prev,
            rotateRightPtCmd_prev,
            fieldRelative,
            getHeading().getDegrees(),
            translation.getX());

    // Set Status of RotatePt Buttons for next loop
    rotateLeftPtCmd_prev = rotateLeftPtCmd;
    rotateRightPtCmd_prev = rotateRightPtCmd;

    // Calculate the Swerve Module States
    SwerveModuleState[] swerveModuleStates =
        SubSys_SwerveDrive_Constants.swerveKinematics.toSwerveModuleStates(
            (fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getHeading())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)),
            rotationPt);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Robot.MaxSpeeds.DriveTrain.DriveTrainMaxSpd);

    // Set Swerve Modules to Calculated States
    swerveModules[0].setDesiredState(swerveModuleStates[0], isOpenLoop); // FL
    swerveModules[1].setDesiredState(swerveModuleStates[1], isOpenLoop); // FR
    swerveModules[2].setDesiredState(swerveModuleStates[2], isOpenLoop); // BL
    swerveModules[3].setDesiredState(swerveModuleStates[3], isOpenLoop); // BR

    SmartDashboard.putNumber("FL_xCmd", swerveModuleStates[0].speedMetersPerSecond);
  }

  // ***** Odometry *****

  /**
   * getHeading Get Swerve Drive Heading in Rotation2d
   *
   * @return Rotation2d Heading of the drive train
   */
  public Rotation2d getHeading() {
    return gyroSubSys.getYawRotation2d();
  }

  /**
   * setGyroYaw set Gyro Yaw Value
   *
   * @param degrees
   */
  public void setYaw(double degrees) {
    gyroSubSys.setYawRelativeToFront(degrees);
  }

  /**
   * getSwerveDriveKinematics Get the Swerve Drive Kinematics
   *
   * @return SwerveDriveKinematics
   */
  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return SubSys_SwerveDrive_Constants.swerveKinematics;
  }

  /**
   * getPose Get the X and Y position of the drivetrain from the SwerveOdometry
   *
   * @return Pose2d X and Y position of the drivetrain
   */
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * setPose Set Pose of Swerve Odometry
   *
   * @param pose Pose2d X and Y position of the drivetrain
   */
  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(getHeading(), getSwerveModulePositions(), pose);
  }

  /** setPoseDriveMtrsToZero Set Pose and Drive Motors to Zero */
  public void setPoseDriveMtrsToZero() {
    // Set Motor Distances to 0
    swerveModules[0].setDriveMotorPos(0.0);
    swerveModules[1].setDriveMotorPos(0.0);
    swerveModules[2].setDriveMotorPos(0.0);
    swerveModules[3].setDriveMotorPos(0.0);
    // Set Drivetrain Pose to 0
    swerveOdometry.resetPosition(
        getHeading(), getSwerveModulePositions(), new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
  }

  /***********************************************************************************/
  /* ***** Private Swerve Methods *****                                              */
  /***********************************************************************************/

  /**
   * getServeModulePositions
   *
   * @return SwerveModulePosition[4] Positions of the Swerve Modules 0- FrontLeft, 1- FrontRight, 2-
   *     BackLeft, 3- BackRight
   */
  private SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];

    swerveModulePositions[0] = swerveModules[0].getPosition();
    swerveModulePositions[1] = swerveModules[1].getPosition();
    swerveModulePositions[2] = swerveModules[2].getPosition();
    swerveModulePositions[3] = swerveModules[3].getPosition();

    return swerveModulePositions;
  }

  // private SwerveModuleState[] getStates(){
  //    SwerveModuleState[] states = new SwerveModuleState[4];
  //
  //    states[0] = swerveModules[0].getState();  // FL
  //    states[1] = swerveModules[1].getState();  // FR
  //    states[2] = swerveModules[2].getState();  // BL
  //    states[3] = swerveModules[3].getState();  // BR
  //
  //    return states;
  // }

  /* Used by SwerveControllerCommand in Auto */
  // public void setModuleStates(SwerveModuleState[] desiredStates) {
  //    SwerveDriveKinematics.desaturateWheelSpeeds(
  //      desiredStates,
  //      Constants.RobotSettings.DriveTrain.DriveTrainMaxSpd);
  //
  //      swerveModules[0].setDesiredState(desiredStates[0], false);  // FL
  //      swerveModules[1].setDesiredState(desiredStates[1], false);  // FR
  //      swerveModules[2].setDesiredState(desiredStates[2], false);  // BL
  //      swerveModules[3].setDesiredState(desiredStates[3], false);  // BR
  //
  // }
}
