// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.CAN;
import frc.robot.Constants.Controllers;
import frc.robot.subsystems.DrivebaseSubsystem;

// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Controllers;
import frc.robot.subsystems.CameraSubsystem;

public class RobotContainer {

  private DrivebaseSubsystem m_drivebase = new DrivebaseSubsystem(
    CAN.LEFT_MOTOR_1, CAN.LEFT_MOTOR_2,
    CAN.RIGHT_MOTOR_1, CAN.RIGHT_MOTOR_2
    // Digital.LEFT_ENCODER_1, Digital.LEFT_ENCODER_2,
    // Digital.RIGHT_ENCODER_1, Digital.RIGHT_ENCODER_2
  );

  private XboxController m_xboxController = new XboxController(Controllers.XBOX);


  public RobotContainer() {
    configureBindings();
  }

    private void configureBindings() {    
    m_drivebase.setDefaultCommand(
      new RunCommand(
        () -> m_drivebase.set(-m_xboxController.getLeftY(), m_xboxController.getLeftX()),
        m_drivebase
      )
    );}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
