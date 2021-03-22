// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrainSubsystem;

public class turnToAngleCommand extends CommandBase {

  float targetAngle;
  float Yaw;
  double inputangle;
  boolean rotateToAngle = false;
  double currentRotationRate;
  drivetrainSubsystem m_subsystem;
 
  public turnToAngleCommand(double inputAngle, drivetrainSubsystem driveSubsystem) {

    m_subsystem = driveSubsystem;
    addRequirements(m_subsystem);
    inputAngle = targetAngle;
     
  }

   
  @Override
  public void initialize() {

    m_subsystem.setAngle(90);

    m_subsystem.setBrake();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
/*
   if (m_subsystem.getNavxYaw(Yaw) <= targetAngle) {

      System.out.printf(" Navx Reading, Running : ");
      System.out.print(m_subsystem.getNavxYaw(Yaw));

    } else {

      end(true);
      System.out.printf(" Navx Reading, Complete : ");
      System.out.print(m_subsystem.getNavxYaw(Yaw));

    }
*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_subsystem.driveStop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
