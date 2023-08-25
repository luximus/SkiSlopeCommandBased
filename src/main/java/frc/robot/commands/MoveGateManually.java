// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GateArm;

/**
 * Move the gate to a set angle.
 */
public class MoveGateManually extends CommandBase {

  private static final double COMPLETION_POSITION_TOLERANCE = 32.0;
  private static final double COMPLETION_VELOCITY_TOLERANCE = 4.0;

  private GateArm gateArm;
  private double targetPosition;

  /** Creates a new MoveGate. */
  public MoveGateManually(GateArm gateArm, double targetPosition) {
    this.gateArm = gateArm;
    addRequirements(gateArm);
    this.targetPosition = targetPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gateArm.moveTo(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gateArm.getArmPositionError() < COMPLETION_POSITION_TOLERANCE && Math.abs(gateArm.getArmVelocity()) < COMPLETION_VELOCITY_TOLERANCE;
  }
}
