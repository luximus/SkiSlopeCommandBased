// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveArcadeStyle;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.MoveGate;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GateArm;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.GateArm.Position;

public class RobotContainer {

  private static final int CONTROLLER_PORT = 0;

  private static final double AUTO_MOVE_SPEED = 0.5;
  private static final double AUTO_MOVE_FORWARD_TIME = 2.5;
  private static final double AUTO_SLIDE_TIME = 2.0;

  private static final Axis DRIVE_FORWARD_AXIS = Axis.kLeftY;
  private static final Axis DRIVE_TURN_AXIS = Axis.kRightX;
  private static final Button GATE_TOGGLE_BUTTON = Button.kA;

  private Drivetrain drivetrain = Drivetrain.getInstance();
  private Gyroscope gyroscope = Gyroscope.getInstance();
  private GateArm gateArm = GateArm.getInstance();

  private XboxController controller = new XboxController(CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(
      new DriveForwardTimed(drivetrain, gyroscope, AUTO_MOVE_SPEED, AUTO_MOVE_FORWARD_TIME),
      new MoveGate(gateArm, GateArm.Position.OPEN),
      Commands.waitSeconds(AUTO_SLIDE_TIME),
      Commands.parallel(
        new MoveGate(gateArm, GateArm.Position.CLOSED),
        new DriveForwardTimed(drivetrain, gyroscope, -AUTO_MOVE_SPEED, AUTO_MOVE_FORWARD_TIME)
      )
    );
  }

  private void configureBindings() {
    configureButtonBindings();

    drivetrain.setDefaultCommand(new DriveArcadeStyle(drivetrain, this::getForwardAxis, this::getTurnAxis, AUTO_MOVE_SPEED, AUTO_MOVE_FORWARD_TIME));
  }

  private void configureButtonBindings() {
    Trigger buttonTrigger = new JoystickButton(controller, GATE_TOGGLE_BUTTON.value);
    buttonTrigger
      .and(() -> gateArm.getArmSetpoint() == Position.OPEN)
      .onTrue(new MoveGate(gateArm, Position.CLOSED));
    buttonTrigger
      .and(() -> gateArm.getArmSetpoint() == Position.CLOSED)
      .onTrue(new MoveGate(gateArm, Position.OPEN));
  }

  private double getForwardAxis() {
    return controller.getRawAxis(DRIVE_FORWARD_AXIS.value);
  }

  private double getTurnAxis() {
    return controller.getRawAxis(DRIVE_TURN_AXIS.value);
  }
}
