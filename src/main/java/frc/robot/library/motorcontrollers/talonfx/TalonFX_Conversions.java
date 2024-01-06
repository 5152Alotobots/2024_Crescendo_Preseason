// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library.motorcontrollers.talonfx;

/** Add your docs here. */
public class TalonFX_Conversions {

  public static final double TALONFX_CNTS_PER_REVS = 2048;
  public static final double REVS_PER_TALONFX_CNTS = 0.00048828125; // 1/TALONFX_CNTS_PER_REVS
  public static final double TALONFX_CNTS_PER_DEGS = 5.688888888889; // TALONFX_CNTS_PER_REVS/360
  public static final double DEGS_PER_TALONFX_CNTS = 0.17578125; // 360/TALONFX_CNTS_PER_REVS
  public static final double CANCODER_CNTS_PER_REVS = 4096;
  public static final double REVS_PER_CANCODER_CNTS = 0.000244140625; // 1/CANCODER_CNTS_PER_REVS
  public static final double CANCODER_CNTS_PER_DEGS = 11.377777777778; // CANCODER_CNTS_PER_REVS/360
  public static final double DEGS_PER_CANCODER_CNTS = 0.087890625; // 360/CANCODER_CNTS_PER_REVS

  /***********************************************************************************/
  /* ***** TalonFX Conversions *****                                                 */
  /***********************************************************************************/

  /**
   * talonFXCntsToRevs
   *
   * @param counts double TalonFX Counts
   * @return double Revolutions
   */
  public static double talonFXCntsToRevs(double counts) {
    // counts to revolutions => 1/2048 = 0.00048828125
    return counts * REVS_PER_TALONFX_CNTS;
  }

  /**
   * revsToTalonFXCnts
   *
   * @param revolutions double revolutions of Mechanism
   * @return TalonFX Counts
   */
  public static double revsToTalonFXCnts(double revolutions) {
    // revolutions to counts => 2048/1
    return revolutions * TALONFX_CNTS_PER_REVS;
  }

  /**
   * talonFXCntsToDegrees
   *
   * @param counts TalonFX Counts
   * @return Degrees of Rotation of Mechanism
   */
  public static double talonFXCntsToDegrees(double counts) {
    // counts to degrees => 360/2048 = 0.17578125
    return counts * DEGS_PER_TALONFX_CNTS;
  }

  /**
   * degreesToTalonFXCnts
   *
   * @param degrees Degrees of rotation of Mechanism
   * @return TalonFX Counts
   */
  public static double degreesToTalonFXCnts(double degrees) {
    // degrees to counts => 2048/360 = 5.688888888889
    return degrees * TALONFX_CNTS_PER_DEGS;
  }

  /**
   * talonFXCntsPer100msToRPM Converts from Talon FX Counts Per 100ms to RPM
   *
   * @param velocitycounts double Counts Per 100ms
   * @return double Talon FX Motor RPM
   */
  public static double talonFXCntsPer100msToRPM(double velocitycounts) {
    // double countsPerSec = velocitycounts*10;
    // double revsPerSec = countsPerSec*REVS_PER_TALONFX_CNTS;
    // double rpm = revsPerSec*60;
    // return rpm;

    // Simplifies to:
    return velocitycounts * 600 * REVS_PER_TALONFX_CNTS;
  }

  /**
   * rpmToTalonFXCntsPer100m Converts to Talon FX Counts Per 100ms from rpm
   *
   * @param rpm double Talon FX Motor RPM
   * @return double Motor Velocity in Counts Per 100m
   */
  public static double rpmToTalonFXCntsPer100ms(double rpm) {
    // double revsPerSec = rpm/60;
    // double countsPerSec = revsPerSec*TALONFX_CNTS_PER_REVS;
    // double velocitycounts = countsPerSec/10;
    // return velocitycounts;

    // Simplifies to:
    // return rpm*TALONFX_CNTS_PER_REVS/600;

    // Mathmatically more efficient
    // 1/600 = 0.00166666666666666666666666666667
    return rpm * TALONFX_CNTS_PER_REVS * 0.0016666667;
  }

  /***********************************************************************************/
  /* ***** CANcoder Conversions *****                                                 */
  /***********************************************************************************/

  /**
   * canCoderCntsToRevs
   *
   * @param counts double CANCoder Counts
   * @return double Revolutions
   */
  public static double canCoderCntsToRevs(double counts) {
    // counts to revolutions => 1/4096
    return counts * REVS_PER_CANCODER_CNTS;
  }

  /**
   * revsToCANCoderCnts
   *
   * @param revolutions double revolutions of Mechanism
   * @return CANCoder Counts
   */
  public static double revsToCANCoderCnts(double revolutions) {
    // revolutions to counts => 2048/1
    return revolutions * CANCODER_CNTS_PER_REVS;
  }

  /**
   * canCoderCntsToDegrees
   *
   * @param counts canCoder Counts
   * @return Degrees of Rotation of Mechanism
   */
  public static double canCoderCntsToDegrees(double counts) {
    // counts to degrees => 360/4096
    return counts * DEGS_PER_CANCODER_CNTS;
  }

  /**
   * degreesToCANCoderCnts
   *
   * @param degrees Degrees of rotation of Mechanism
   * @return CANCoder Counts
   */
  public static double degreesToCANCoderCnts(double degrees) {
    // degrees to counts => 2048/360 = 5.688888888889
    return degrees * CANCODER_CNTS_PER_DEGS;
  }

  /**
   * talonFXToDegrees_wGearRatio
   *
   * @param counts TalonFX Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  // public static double talonFXToDegrees_wGearRatio(double counts, double gearRatio) {
  //    // counts to degrees => 360/4096 = 0.087890625
  //    return counts * 0.087890625 / gearRatio;
  // }

  /**
   * degreesToTalonFX_wGearRatio
   *
   * @param degrees Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return TalonFX Counts
   */
  // public static double degreesToTalonFX_wGearRatio(double degrees, double gearRatio) {
  //    // degrees to counts => 4096/360 = 11.37778
  //    double counts =  degrees * 11.37778 / gearRatio;
  //    return counts;
  // }

  /**
   * talonFXToRevs Convert from TalonFX Sensor Counts to Revolutions
   *
   * @param counts double TalonFX Sensor Counts
   * @return double Revolutions
   */
  // public static double talonFXToRevs (double counts){
  //    return counts*TALONFX_CNTS_TO_REVS;
  // }

  /**
   * talonFXToRevs_GearRatio Convert from TalonFX Counts to Revolutions through a gearbox
   *
   * @param counts double TalonFX Sensor Counts
   * @param gearRatio double GearRatio
   * @return double Revolutions
   */
  // public static double talonFXToRevs_GearRatio (double counts, double gearRatio){
  //    return counts*TALONFX_CNTS_TO_REVS*gearRatio;
  // }

}
