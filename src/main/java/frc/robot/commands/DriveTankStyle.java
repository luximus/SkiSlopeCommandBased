// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Drive the robot, tank style, with the given suppliers for left and right inputs.
 * 
 * @author Ivan Post
 * @version 0.0.1
 */
public class DriveTankStyle extends CommandBase {
  public static interface AxisValueProducer {
    public double get();
  }

  private Drivetrain drivetrain;
  private DoubleSupplier leftAxisSupplier, rightAxisSupplier;
  private double leftMaxSpeed, rightMaxSpeed;

  /** Creates a new DriveTankStyle. */
  public DriveTankStyle(Drivetrain drivetrain, DoubleSupplier leftAxisSupplier, DoubleSupplier rightAxisSupplier, double leftMaxSpeed, double rightMaxSpeed) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.leftAxisSupplier = leftAxisSupplier;
    this.rightAxisSupplier = rightAxisSupplier;
    this.leftMaxSpeed = leftMaxSpeed;
    this.rightMaxSpeed = rightMaxSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.tankDrive(leftMaxSpeed * leftAxisSupplier.getAsDouble(), rightMaxSpeed * rightAxisSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
