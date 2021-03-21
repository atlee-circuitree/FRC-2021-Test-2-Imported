package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrainSubsystem;

public class driveForwardCommand extends CommandBase {

  double encoderTarget;
  double targetSpeed;
  double encoderReadingLeft;
  double encoderReadingRight;
  drivetrainSubsystem m_subsystem;

  public driveForwardCommand(double targetValue, double speed, drivetrainSubsystem driveSubsystem) {

    m_subsystem = driveSubsystem;
    addRequirements(m_subsystem);
    encoderTarget = targetValue;
    targetSpeed = speed;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_subsystem.resetEncoders();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (-encoderReadingRight <= Math.abs(encoderTarget)) {

    encoderReadingLeft = m_subsystem.getLeftEncoder();
    encoderReadingRight = m_subsystem.getRightEncoder();

    System.out.printf(" Encoder Reading, Running : ");
    System.out.print(-encoderReadingRight);
     
    m_subsystem.driveStraight(targetSpeed);

    } else {

    System.out.print(" Encoder Reading, Complete : ");
    System.out.print(encoderReadingRight);
     
    m_subsystem.driveStop();

    end(true);

    }

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