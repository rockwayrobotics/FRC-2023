// ROBOTCONTAINER
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

  private CameraSubsystem m_CameraSubsystem = new CameraSubsystem(m_xboxController);

  private ShuffleboardTab m_autoSelectorTab = Shuffleboard.getTab("Auto Selection");

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public final GenericEntry autoSpeed;

  public final Command m_driveForever;
  public final Command m_driveForeverSlow;


  public RobotContainer() {
    configureBindings();
    var autoTab = Shuffleboard.getTab("Auto");
    autoSpeed = autoTab.addPersistent("Max Speed", 1).getEntry();
    
    m_driveForever = new DriveDistance(m_drivebase, () -> autoSpeed.getDouble(0.5));
    m_driveForeverSlow = new DriveDistance(m_drivebase, () -> 0.1);

    m_chooser.setDefaultOption("Drive Forever", m_driveForever);
    m_chooser.addOption("Drive Forever Slow", m_driveForeverSlow);
    autoTab.add(m_chooser);

    var auto1 = new SetDriveScaleCommand(m_drivebase, 0);
    var auto2 = new SetDriveScaleCommand(m_drivebase, 0.5);
    var auto3 = new SetDriveScaleCommand(m_drivebase, 0.7);

    m_chooser.addOption("0 Speed", auto1);
    m_chooser.addOption("0.5 Speed", auto2);
    m_chooser.addOption("0.7 Speed", auto3);
    m_autoSelectorTab.add(m_chooser);

    m_drivebase.setDefaultCommand(new DriveCommand(() -> m_xboxController.getLeftY(), () -> m_xboxController.getLeftX(), m_drivebase));

    Shuffleboard.getTab("Subsystems").add(m_CameraSubsystem);
    Shuffleboard.getTab("Subsystems").add(m_drivebase);
  }

  private void configureBindings() {
    final JoystickButton rightBumper = new JoystickButton(m_xboxController, XboxController.Button.kLeftBumper.value);
    rightBumper.onTrue(new SetDriveScaleCommand(m_drivebase, Drive.SLOMODE_SCALE));
    rightBumper.onFalse(new SetDriveScaleCommand(m_drivebase, 1));

    final JoystickButton buttonA = new JoystickButton(m_xboxController, XboxController.Button.kA.value);
    buttonA.onTrue(new AlignRobotToTarget(m_drivebase, m_CameraSubsystem));
  }

  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }
}
