// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The arm for the gate that opens to release a bucket.
 * 
 * @version 0.0.1
 */
public final class GateArm extends SubsystemBase {

  private static final int MOTOR_CONTROLLER_CAN_ID = 2;
  private static final int MOTOR_CONTROLLER_CLOSED_LOOP_PEAK_OUTPUT = 3;
  private static final double MOTOR_CONTROLLER_CLOSED_LOOP_RAMP_RATE = 0.4;
  private static final boolean USE_MANUAL_PID_GAINS = false;
  private static final double MOTOR_CONTROLLER_PID_POSITION_GAIN = 0.0;
  private static final double MOTOR_CONTROLLER_PID_INTEGRAL_GAIN = 0.0;
  private static final double MOTOR_CONTROLLER_PID_DERIVATIVE_GAIN = 0.0;
  private static final int MOTOR_ENCODER_RESET_TIMEOUT_MILLISECONDS = 15;
  private static final double OPEN_POSITION = 90.0;
  private static final double CLOSED_POSITION = 0.0;

  private static final GateArm instance = new GateArm();

  private TalonFX motorController;
  private TalonFXSensorCollection motorEncoder;
  private Position setpoint = Position.CLOSED;

  /**
   * Known positions of the gate arm.
   */
  public static enum Position {
    OPEN, CLOSED
  }

  public static GateArm getInstance() {
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Get the position of the gate arm in the units of its encoder.
   */
  public double getArmPosition() {
    return motorController.getSelectedSensorPosition();
  }

  /**
   * Get the velocity of the gate arm in the units of its encoder.
   */
  public double getArmVelocity() {
    return motorController.getSelectedSensorVelocity();
  }

  /**
   * Get the distance between the gate arm and its setpoint in the units of its encoder.
   */
  public double getArmPositionError() {
    return motorController.getClosedLoopError();
  }

  public Position getArmSetpoint() {
    return setpoint;
  }

  /**
   * Reset the motor's encoder to zero.
   */
  public void resetEncoder() {
    motorEncoder.setIntegratedSensorPosition(0, MOTOR_ENCODER_RESET_TIMEOUT_MILLISECONDS);
  }

  /**
   * Move the gate to the open position.
   */
  public void moveToOpenPosition() {
    moveTo(OPEN_POSITION);
  }

  /**
   * Move the gate to the closed position.
   */
  public void moveToClosedPosition() {
    moveTo(CLOSED_POSITION);
  }

  /**
   * Move the gate to the provided position.
   * @param position the target position of the gate
   */
  public void moveTo(Position position) {
    switch (position) {
      case OPEN:
        moveToOpenPosition();
        break;
      case CLOSED:
        moveToClosedPosition();
        break;
    }
  }

  /**
   * Move the gate to the given position.
   * @param angle the angle to move to
   */
  public void moveTo(double position) {
    motorController.set(TalonFXControlMode.Position, position);
  }

  /** Creates a new GateArm. */
  private GateArm() {
    motorController = new TalonFX(MOTOR_CONTROLLER_CAN_ID);

    motorController.configClosedLoopPeakOutput(0, MOTOR_CONTROLLER_CLOSED_LOOP_PEAK_OUTPUT);
    motorController.configClosedloopRamp(MOTOR_CONTROLLER_CLOSED_LOOP_RAMP_RATE);

    if (USE_MANUAL_PID_GAINS) {
      motorController.config_kP(0, MOTOR_CONTROLLER_PID_POSITION_GAIN);
      motorController.config_kI(0, MOTOR_CONTROLLER_PID_INTEGRAL_GAIN);
      motorController.config_kD(0, MOTOR_CONTROLLER_PID_DERIVATIVE_GAIN);
    }

    motorEncoder = new TalonFXSensorCollection(motorController);

    moveTo(Position.CLOSED);
  }
}
