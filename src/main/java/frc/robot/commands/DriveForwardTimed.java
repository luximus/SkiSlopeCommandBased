// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyroscope;

public class DriveForwardTimed extends CommandBase {
  private Drivetrain drivetrain;
  private Gyroscope gyro;
  private PIDController pidController;
  private double forwardSpeed;
  private double timeSeconds;

  private Timer timer = new Timer();

  /** Creates a new DriveForwardTimed. */
  public DriveForwardTimed(Drivetrain drivetrain, Gyroscope gyro, double positionGain, double integralGain, double derivativeGain, double forwardSpeed, double timeSeconds) {
    this.drivetrain = drivetrain;
    this.gyro = gyro;
    addRequirements(drivetrain, gyro);
    this.pidController = new PIDController(positionGain, integralGain, derivativeGain);
    this.forwardSpeed = forwardSpeed;
    this.timeSeconds = timeSeconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    pidController.setSetpoint(gyro.getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(forwardSpeed, pidController.calculate(gyro.getAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > timeSeconds;
  }
}
