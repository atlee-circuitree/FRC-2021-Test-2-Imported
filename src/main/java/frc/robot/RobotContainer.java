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

public class RobotContainer {

  //217.2944297082
 
  //constants

  public static Constants m_constants;

  //subsystems
   
  private final drivetrainSubsystem m_drivetrainSubsystem = new drivetrainSubsystem();

  //commands

  public Command GenerateEncoderDriveCommand(double inches, double speed)
  {

    
      double encoder = inches * 217.2944297082;

      Command m_driveStraightUntilEncoderValueCommand = new driveForwardCommand(encoder, speed, m_drivetrainSubsystem);

      return m_driveStraightUntilEncoderValueCommand;
      
  }

  public void driveSetup() {

    SpeedControllerGroup leftDrive = new SpeedControllerGroup(m_drivetrainSubsystem.leftFrontMotor, m_drivetrainSubsystem.leftBackMotor);
    SpeedControllerGroup rightDrive = new SpeedControllerGroup(m_drivetrainSubsystem.rightFrontMotor, m_drivetrainSubsystem.rightBackMotor); 

    m_drivetrainSubsystem.driveSetup(leftDrive, rightDrive);

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
   
    return GenerateEncoderDriveCommand(60, .3);

  }

}