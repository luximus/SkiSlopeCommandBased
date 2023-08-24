// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A robot drivetrain.
 * 
 * This drivetrain implementation uses a differential drive.
 * 
 * @author Ivan Post
 * @version 0.0.1
 */
public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  public Drivetrain() {}

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

  }

  /**
   * Drive the robot with a tank drive style.
   * @param leftMotorSpeed the speed of the left motor
   * @param rightMotorSpeed the speed of the right motor
   */
  public void tankDrive(double leftMotorSpeed, double rightMotorSpeed) {

  }

  /**
   * Drive the robot with a curvature drive style.
   * @param forwardSpeed
   * @param turnSpeed
   */
  public void curvatureDrive(double forwardSpeed, double turnSpeed) {

  }

  /**
   * Stop the robot from moving.
   */
  public void stop() {

  }
}
