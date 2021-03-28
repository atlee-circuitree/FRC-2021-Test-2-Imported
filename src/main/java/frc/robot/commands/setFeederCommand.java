package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrainSubsystem;

public class setFeederCommand extends CommandBase {

  drivetrainSubsystem m_subsystem;
  double runSpeed;
  boolean finishCommand = false;

  public setFeederCommand(double speed, drivetrainSubsystem driveSubsystem) {

    m_subsystem = driveSubsystem;
    addRequirements(m_subsystem);
    runSpeed = speed;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //m_subsystem.resetEncoders();

    m_subsystem.setBrake();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_subsystem.setFeeder(runSpeed); 

    end(true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_subsystem.driveStop();

    finishCommand = true;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (finishCommand = false) {

      return false;

    } else {

      return true;

    }
    
  }

}