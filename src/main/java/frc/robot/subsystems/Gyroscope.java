// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * 
 */
public final class Gyroscope extends SubsystemBase {

  private static final Gyroscope instance = new Gyroscope();

  private ADXRS450_Gyro gyro;

  public static Gyroscope getInstance() {
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Get the angle of the gyroscope.
   */
  public double getAngle() {
    return gyro.getAngle();
  }

  /**
   * Get the angular velocity of the gyroscope.
   */
  public double getRate() {
    return gyro.getRate();
  }

  /**
   * Reset the gyroscope to the zero angle.
   */
  public void reset() {
    gyro.reset();
  }

  /**
   * Calibrate the gyroscope. Do not call this method if the robot is moving.
   */
  public void calibrate() {
    gyro.calibrate();
  }

  /** Creates a new Gyrometer. */
  private Gyroscope() {
    gyro = new ADXRS450_Gyro();
  }

  private Gyroscope(Port port) {
    gyro = new ADXRS450_Gyro(port);
  }
}
