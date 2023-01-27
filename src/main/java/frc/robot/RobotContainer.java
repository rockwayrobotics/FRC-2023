// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Digital;
import frc.robot.Constants.Pneumatics;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Controllers;
import frc.robot.subsystems.LifterSubsystem;

public class RobotContainer {
  private LifterSubsystem m_lifterSubsystem = new LifterSubsystem(
    Pneumatics.CLAW_EXTEND, 
    Pneumatics.CLAW_RETRACT, 
    CAN.CLAW_ELEVATOR,
    Digital.TOP_ELEVATOR_LIMIT,
    Digital.BOTTOM_ELEVATOR_LIMIT
  );

  private XboxController m_xboxController = new XboxController(Controllers.XBOX);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_lifterSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_lifterSubsystem.moveElevator(m_xboxController.getRightY()),
        m_lifterSubsystem)
    );
  }
  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
