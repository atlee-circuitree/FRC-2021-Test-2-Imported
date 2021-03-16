/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainSubsystem;
import frc.robot.commands.driveCommand;
import frc.robot.Constants;
import frc.robot.commands.driveForwardCommand;
import frc.robot.commands.turnToAngleCommand;

public class RobotContainer {

  // 217.2944297082 Old Wheel Dia x Pi
  // 78.9168074582 New Wheel Dia x Pi
  // 26.523624420166016 Very Rough 5 feet estamate 
  // COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
  // CPI = (42 * 10.75) / (78.9168074582)

  //constants

  public static Constants m_constants;

  //subsystems
   
  private final drivetrainSubsystem m_drivetrainSubsystem = new drivetrainSubsystem();

  //commands

  public Command GenerateEncoderDriveCommand(double inches, double speed)
  {

      double PPR;
      double GearReduction;
      double WheelDiameter;
      double Pi;
      double CPI;

      PPR = 42;
      GearReduction = 10.75;
      WheelDiameter = 8;
      Pi = 3.1415;
      CPI = (PPR * GearReduction) / (WheelDiameter * Pi);
      //451.5, 25.132


      double encoder = (inches / 12) * CPI;

      Command m_driveStraightUntilEncoderValueCommand = new driveForwardCommand(encoder, speed, m_drivetrainSubsystem);

      return m_driveStraightUntilEncoderValueCommand;
      
  }

  public Command GenerateTurnCommand(double angle) {

      Command m_turnToAngleCommand = new turnToAngleCommand(angle, m_drivetrainSubsystem);

      return m_turnToAngleCommand;

  }

  public void driveSetup() {

    CANSparkMax leftFrontMotor = new CANSparkMax(Constants.driveFrontLeftMotor, MotorType.kBrushless);
    CANSparkMax leftBackMotor = new CANSparkMax(Constants.driveBackLeftMotor, MotorType.kBrushless);

    CANSparkMax rightFrontMotor = new CANSparkMax(Constants.driveFrontRightMotor, MotorType.kBrushless);
    CANSparkMax rightBackMotor = new CANSparkMax(Constants.driveBackRightMotor, MotorType.kBrushless);

    m_drivetrainSubsystem.driveSetup(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);

  }

  public RobotContainer() {

    driveSetup();

    configureButtonBindings();

  }

  public void teleopInit() {

    driveCommand m_driveCommand = new driveCommand(Constants.Xbox1, m_drivetrainSubsystem);

    m_drivetrainSubsystem.setDefaultCommand(m_driveCommand);

  }

  private void configureButtonBindings() {


  }

  public Command getAutonomousCommand() {
   
    //return GenerateEncoderDriveCommand(36, .1);
    return GenerateTurnCommand(90);  
  }

}