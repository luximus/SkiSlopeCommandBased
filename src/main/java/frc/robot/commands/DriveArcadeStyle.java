// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Drive the robot, arcade style, with the given suppliers for linear and rotational inputs.
 * 
 * @author Ivan Post
 * @version 0.0.1
 */
public class DriveArcadeStyle extends CommandBase {

  private Drivetrain drivetrain;
  private DoubleSupplier forwardAxisSupplier, turnAxisSupplier;
  private double forwardMaxSpeed, turnMaxSpeed;

  /** Creates a new DriveArcadeStyle. */
  public DriveArcadeStyle(Drivetrain drivetrain, DoubleSupplier forwardAxisSupplier, DoubleSupplier turnAxisSupplier, double forwardMaxSpeed, double turnMaxSpeed) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.forwardAxisSupplier = forwardAxisSupplier;
    this.turnAxisSupplier = turnAxisSupplier;
    this.forwardMaxSpeed = forwardMaxSpeed;
    this.turnMaxSpeed = turnMaxSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(forwardMaxSpeed * forwardAxisSupplier.getAsDouble(), turnMaxSpeed * turnAxisSupplier.getAsDouble());
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
