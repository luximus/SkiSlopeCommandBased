// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A robot drivetrain singleton.
 * 
 * This drivetrain implementation uses a differential drive.
 * 
 * @version 0.0.1
 */
public final class Drivetrain extends SubsystemBase {

  private static final int LEFT_MOTOR_CONTROLLER_CAN_ID = 0;
  private static final int RIGHT_MOTOR_CONTROLLER_CAN_ID = 1;
  private static final MotorType MOTOR_TYPE = MotorType.kBrushed;
  private static final double RAMP_RATE = 0.4;

  private static final Drivetrain instance = new Drivetrain();

  CANSparkMax leftMotorController, rightMotorController;
  
  DifferentialDrive differentialDrive;

  public static Drivetrain getInstance() {
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Drive the robot with an arcade drive style.
   * @param forwardSpeed the forward speed of the robot
   * @param turnSpeed the turn speed of the robot
   */
  public void arcadeDrive(double forwardSpeed, double turnSpeed) {
    differentialDrive.arcadeDrive(forwardSpeed, turnSpeed);
  }

  /**
   * Drive the robot with a tank drive style.
   * @param leftMotorSpeed the speed of the left motor
   * @param rightMotorSpeed the speed of the right motor
   */
  public void tankDrive(double leftMotorSpeed, double rightMotorSpeed) {
    differentialDrive.tankDrive(leftMotorSpeed, rightMotorSpeed);
  }

  /**
   * Drive the robot with a curvature drive style.
   * @param forwardSpeed the forward speed of the robot
   * @param turnSpeed the turn speed of the robot
   */
  public void curvatureDrive(double forwardSpeed, double turnSpeed) {
    differentialDrive.curvatureDrive(forwardSpeed, turnSpeed, true);
  }

  /**
   * Stop the robot from moving.
   */
  public void stop() {
    leftMotorController.set(0);
    rightMotorController.set(0);
  }

  /** Creates a new Drivetrain. */
  private Drivetrain() {
    leftMotorController = new CANSparkMax(LEFT_MOTOR_CONTROLLER_CAN_ID, MOTOR_TYPE);
    rightMotorController = new CANSparkMax(RIGHT_MOTOR_CONTROLLER_CAN_ID, MOTOR_TYPE);
    differentialDrive = new DifferentialDrive(leftMotorController, rightMotorController);

    leftMotorController.setOpenLoopRampRate(RAMP_RATE);
    rightMotorController.setOpenLoopRampRate(RAMP_RATE);
  }
}
