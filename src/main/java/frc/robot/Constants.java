/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public final class Constants {

  // Map all the motors / pnumatics here

  // Drivetrain

  // 3/19/21 Changed the can ID number in order to make them more correct

  public static int driveFrontLeftMotor = 3;
  public static int driveFrontRightMotor = 1; //1
  public static int driveBackLeftMotor = 2;
  public static int driveBackRightMotor = 4;

  static XboxController Xbox1 = new XboxController(0);
   
}
