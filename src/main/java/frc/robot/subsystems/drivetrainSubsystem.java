/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// CPR 4096

// Wheel Cir. 18.85

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class drivetrainSubsystem extends SubsystemBase {
   
  DifferentialDrive robotDrive;
 
  public CANSparkMax leftFrontMotor = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax rightFrontMotor = new CANSparkMax(2, MotorType.kBrushless);
  public CANSparkMax leftBackMotor = new CANSparkMax(3, MotorType.kBrushless);
  public CANSparkMax rightBackMotor = new CANSparkMax(4, MotorType.kBrushless);

  SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftFrontMotor, leftBackMotor);
  SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightFrontMotor, rightBackMotor);

  CANEncoder leftEncoder;
  CANEncoder rightEncoder;

  PIDController turnController;

  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;

  static final double kToleranceDegrees = 2.0f;

  final static int frontLeftChannel = 2;
  final static int rearLeftChannel = 3;
  final static int frontRightChannel = 1;
  final static int rearRightChannel = 0;

  public void driveSetup(SpeedControllerGroup leftDrive, SpeedControllerGroup rightDrive){

  leftEncoder = leftFrontMotor.getEncoder();
  rightEncoder = rightFrontMotor.getEncoder();

  robotDrive = new DifferentialDrive(leftDrive, rightDrive);

  resetEncoders();

  }

  public void driveRobot(Double X, double Y) {

    robotDrive.arcadeDrive(-Y, X, true);
  
  }

  public void driveStraight(double Power) {

    leftDrive.set(-Power);
    rightDrive.set(Power);

  }

  public void correctLeft(double Power) {

    leftDrive.set(Power - .1);
    rightDrive.set(Power);

  }

  public void correctRight(double Power) {

    leftDrive.set(Power);
    rightDrive.set(Power - .1);

  }

  public void driveStop() {

    leftDrive.stopMotor();
    rightDrive.stopMotor();

  }

  public void resetEncoders() {

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

  }

  public double getLeftEncoder() {

    return leftEncoder.getPosition();

  }

  public double getRightEncoder() {

    return rightEncoder.getPosition();

  }

  public double getAverageEncoderDistance() {

    return (rightEncoder.getPosition() + leftEncoder.getPosition()) / 2.0;
     
  }

  public drivetrainSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}