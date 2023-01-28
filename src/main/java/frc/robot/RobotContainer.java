// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.DriveDistance;

public class RobotContainer {

  private DrivebaseSubsystem m_drivebase = new DrivebaseSubsystem(
    CAN.LEFT_MOTOR_1, CAN.LEFT_MOTOR_2,
    CAN.RIGHT_MOTOR_1, CAN.RIGHT_MOTOR_2,
    Digital.LEFT_ENCODER_1, Digital.LEFT_ENCODER_2,
    Digital.RIGHT_ENCODER_1, Digital.RIGHT_ENCODER_2
  );

  private XboxController m_xboxController = new XboxController(Gamepads.XBOX);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public final Command m_driveForever = new DriveDistance(m_drivebase, 0.5);
  public final Command m_driveForeverSlow = new DriveDistance(m_drivebase, 0.1);

  public RobotContainer() {
    configureBindings();

    m_chooser.addOption("Drive Forever", m_driveForever);
    m_chooser.addOption("Drive Forever Slow", m_driveForeverSlow);

    Shuffleboard.getTab("Auto").add(m_chooser);

    m_drivebase.setDefaultCommand(new DriveCommand(() -> m_xboxController.getLeftY(), () -> m_xboxController.getLeftX(), m_drivebase));
  }

  private void configureBindings() {
    final JoystickButton rightBumper = new JoystickButton(m_xboxController, XboxController.Button.kLeftBumper.value);
    rightBumper.onTrue(new SetDriveScaleCommand(m_drivebase, Drive.SLOMODE_SCALE));
    rightBumper.onFalse(new SetDriveScaleCommand(m_drivebase, 1));
  }

  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }
}
