// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GateArm extends SubsystemBase {
  /** Creates a new GateArm. */
  public GateArm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getArmPosition() {
    return 0.0;
  }

  public double getArmVelocity() {
    return 0.0;
  }

  public void moveTo(double angle) {

  }
}
